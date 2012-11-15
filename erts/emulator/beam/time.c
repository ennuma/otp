/*
 * %CopyrightBegin%
 * 
 * Copyright Ericsson AB 1996-2011. All Rights Reserved.
 * 
 * The contents of this file are subject to the Erlang Public License,
 * Version 1.1, (the "License"); you may not use this file except in
 * compliance with the License. You should have received a copy of the
 * Erlang Public License along with this software. If not, it can be
 * retrieved online at http://www.erlang.org/.
 * 
 * Software distributed under the License is distributed on an "AS IS"
 * basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See
 * the License for the specific language governing rights and limitations
 * under the License.
 * 
 * %CopyrightEnd%
 */

/*
 * TIMING WHEEL
 * 
 * Timeouts kept in an wheel. A timeout is measured relative to the
 * current slot (tiw_pos) in the wheel, and inserted at slot 
 * (tiw_pos + timeout) % TIW_SIZE. Each timeout also has a count
 * equal to timeout/TIW_SIZE, which is needed since the time axis
 * is wrapped arount the wheel. 
 *
 * Several slots may be processed in one operation. If the number of
 * slots is greater that the wheel size, the wheel is only traversed
 * once,
 *
 * The following example shows a time axis where there is one timeout
 * at each "tick", and where 1, 2, 3 ... wheel slots are released in
 * one operation. The notation "<x" means "release all items with
 * counts less than x". 
 *
 * Size of wheel: 4
 * 
 *   --|----|----|----|----|----|----|----|----|----|----|----|----|----
 *    0.0  0.1  0.2  0.3  1.0  1.1  1.2  1.3  2.0  2.1  2.2  2.3  3.0
 * 
 * 1   [    )
 *     <1  0.1  0.2  0.3  0.0  1.1  1.2  1.3  1.0  2.1  2.2  2.3  2.0
 * 
 * 2   [         )
 *     <1   <1  0.2  0.3  0.0  0.1  1.2  1.3  1.0  1.1  2.2  2.3  2.0
 * 
 * 3   [              )
 *     <1   <1   <1  0.3  0.0  0.1  0.2  1.3  1.0  1.1  1.2  2.3  2.0
 * 
 * 4   [                   )
 *     <1   <1   <1   <1  0.0  0.1  0.2  0.3  1.0  1.1  1.2  1.3  2.0
 * 
 * 5   [                        )
 *     <2   <1   <1   <1.      0.1  0.2  0.3  0.0  1.1  1.2  1.3  1.0
 * 
 * 6   [                             )
 *     <2   <2   <1   <1.           0.2  0.3  0.0  0.1  1.2  1.3  1.0
 * 
 * 7   [                                  )
 *     <2   <2   <2   <1.                0.3  0.0  0.1  0.2  1.3  1.0
 * 
 * 8   [                                       )   
 *     <2   <2   <2   <2.                     0.0  0.1  0.2  0.3  1.0
 * 
 * 9   [                                            )
 *     <3   <2   <2   <2.                          0.1  0.2  0.3  0.0
 * 
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include "sys.h"
#include "erl_vm.h"
#include "global.h"

#ifdef ERTS_ENABLE_LOCK_CHECK
#define ASSERT_NO_LOCKED_LOCKS		erts_lc_check_exact(NULL, 0)
#else
#define ASSERT_NO_LOCKED_LOCKS
#endif

/* BEGIN tiw_lock protected variables 
**
** The individual timer cells in tiw are also protected by the same mutex.
*/

struct tiw_head {
    ErlTimer* head;
    ErlTimer* tail;
};

#ifdef SMALL_MEMORY
#define DEF_TIW_SIZE 8192
#else
#define DEF_TIW_SIZE 65536	/* timing wheel size (should be a power of 2) */
#endif
static Uint tiw_size;
#define TIW_SIZE tiw_size

#if defined(ERTS_SMP)
#define DEF_TIW_INSTANCES erts_no_schedulers
#else
#define DEF_TIW_INSTANCES 1
#endif

typedef struct tiw_s {
    erts_smp_mtx_t tiw_lock;
    struct tiw_head* tiw;	/* the timing wheel, allocated in init_time() */
    Uint tiw_pos;		/* current position in wheel */
    Uint tiw_count;		/* current count */
    Uint tiw_nto;		/* number of timeouts in wheel */
    erts_smp_atomic32_t tiw_min;
    ErlTimer *tiw_min_ptr;
} ErlTiwInstance;

static ErlTiwInstance* tiwi;
static Uint ntiwi;

/* END tiw_lock protected variables */

/* Actual interval time chosen by sys_init_time() */
static int itime; /* Constant after init */

erts_smp_atomic32_t do_time;	/* set at clock interrupt */
static ERTS_INLINE erts_short_time_t do_time_read(void)
{
    return erts_smp_atomic32_read_acqb(&do_time);
}

static ERTS_INLINE erts_short_time_t do_time_update(void)
{
    return do_time_read();
}

static ERTS_INLINE void do_time_init(void)
{
    erts_smp_atomic32_init_nob(&do_time, 0);
}

static ERTS_INLINE erts_aint32_t read_tiw_min (ErlTiwInstance* tiwi)
{
    return erts_smp_atomic32_read_acqb(&tiwi->tiw_min);
}

static ERTS_INLINE void set_tiw_min (ErlTiwInstance* tiwi, erts_aint32_t val)
{
    erts_smp_atomic32_set_acqb(&tiwi->tiw_min, val);
}

static ERTS_INLINE void reset_tiw_min (ErlTiwInstance* tiwi)
{
    set_tiw_min(tiwi, -1);
}

/* get the time (in units of itime) to the next timeout,
   or -1 if there are no timeouts                     */

static erts_short_time_t next_time_internal(ErlTiwInstance* tiwi) /* PRE: tiw_lock taken by caller */
{
    int i, tm, nto;
    Uint32 min;
    ErlTimer* p;
    erts_short_time_t dt;
  
    if (tiwi->tiw_nto == 0)
	return -1;	/* no timeouts in wheel */

    if (tiwi->tiw_min_ptr) {
	min = read_tiw_min(tiwi);
	dt  = do_time_read();
	return ((min >= dt) ? (min - dt) : 0);
    }
  
    /* start going through wheel to find next timeout */
    tm = nto = 0;
    min = (Uint32) -1;	/* max Uint32 */
    i = tiwi->tiw_pos;
    do {
	if ((p = tiwi->tiw[i].head) != NULL) {
	    if (p->count <= tiwi->tiw_count) {
		/* found next timeout */
		dt = do_time_read();
		tiwi->tiw_min_ptr = p;
		set_tiw_min(tiwi, tm);
		return ((tm >= dt) ? (tm - dt) : 0);
	    } else {
		/* keep shortest time in 'min' */
		if (tm + (p->count-tiwi->tiw_count)*TIW_SIZE < min) {
		    min = tm + (p->count-tiwi->tiw_count)*TIW_SIZE;
		    tiwi->tiw_min_ptr = p;
		    set_tiw_min(tiwi, min);
		}
	    }
	}
	tm++;
	i = (i + 1) % TIW_SIZE;
    } while (i != tiwi->tiw_pos);
    dt = do_time_read();
    if (min <= (Uint32) dt)
	return 0;
    if ((min - (Uint32) dt) > (Uint32) ERTS_SHORT_TIME_T_MAX)
	return ERTS_SHORT_TIME_T_MAX;
    return (erts_short_time_t) (min - (Uint32) dt);
}

static void remove_timer(ErlTiwInstance* tiwi, ErlTimer *p) {
    /* first */
    if (!p->prev) {
	tiwi->tiw[p->slot].head = p->next;
	if(p->next)
	    p->next->prev = NULL;
    } else {
	p->prev->next = p->next;
    }

    /* last */
    if (!p->next) {
	tiwi->tiw[p->slot].tail = p->prev;
	if (p->prev)
	    p->prev->next = NULL;
    } else {
	p->next->prev = p->prev;
    }

    p->next = NULL;
    p->prev = NULL;
    /* Make sure cancel callback isn't called */
    p->active = 0;
    tiwi->tiw_nto--;
}

/* Private export to erl_time_sup.c */
erts_short_time_t erts_next_time(void)
{
    erts_short_time_t ret, dt, min;
    int i;

    min = ERTS_SHORT_TIME_T_MAX;
    dt = do_time_update();
    for (i = 0; i < ntiwi; ++i) {
	ret = read_tiw_min(tiwi+i);
	if (ret < 0) {
	    erts_smp_mtx_lock(&tiwi[i].tiw_lock);
	    ret = next_time_internal(tiwi+i);
	    erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
	} else {
	    ret = (ret < dt) ? 0 : ret - dt;
	}
	if (!ret) {
	    return 0;
	}
	if (ret > 0 && ret < min) {
	    min = ret;
	}
    }
    return min;
}

static ERTS_INLINE void bump_timer_internal(ErlTiwInstance* tiwi, erts_short_time_t dt) /* PRE: tiw_lock is write-locked */
{
    Uint keep_pos;
    Uint extra;
    ErlTimer *p, **prev, *timeout_head, **timeout_tail;
    Uint dtime = (Uint) dt;  

    /* no need to bump the position if there aren't any timeouts */
    if (tiwi->tiw_nto == 0) {
	erts_smp_mtx_unlock(&tiwi->tiw_lock);
	return;
    }

    /* if do_time > TIW_SIZE we don't want to go around more than once */
    if (dtime >= TIW_SIZE) {
	tiwi->tiw_count += (Uint)(dtime / TIW_SIZE) - 1;
	dtime = TIW_SIZE;
	extra = 1;
    } else {
	extra = 0;
    }
    keep_pos = (tiwi->tiw_pos + dtime) % TIW_SIZE;
  
    timeout_head = NULL;
    timeout_tail = &timeout_head;
    while (dtime > 0) {
	/* this is to bump the count with the right amount */
	/* when dtime >= TIW_SIZE */
	if (tiwi->tiw_pos == keep_pos) extra = 0;
	prev = &tiwi->tiw[tiwi->tiw_pos].head;
	while ((p = *prev) != NULL) {
	    ASSERT( p != p->next);
	    if (p->count > tiwi->tiw_count+extra) {
		/* no more ripe timeouts in this slot */
		break;
	    }
	    /* reset min time if this timer */
	    if (tiwi->tiw_min_ptr == p) {
		tiwi->tiw_min_ptr = NULL;
		reset_tiw_min(tiwi);
	    }

	    /* Remove from list */
	    remove_timer(tiwi, p);
	    *timeout_tail = p;	/* Insert in timeout queue */
	    timeout_tail = &p->next;
	}
	if (++tiwi->tiw_pos == TIW_SIZE) {
	    tiwi->tiw_pos = 0;
	    ++tiwi->tiw_count;
	}
	dtime--;
    }
    tiwi->tiw_pos = keep_pos;
    if (tiwi->tiw_min_ptr)
    {
	Uint min = read_tiw_min(tiwi);
	set_tiw_min(tiwi, (min > dt) ? min - dt : 0);
    }
    
    erts_smp_mtx_unlock(&tiwi->tiw_lock);
    
    /* Call timedout timers callbacks */
    while (timeout_head) {
	p = timeout_head;
	timeout_head = p->next;
	/* Here comes hairy use of the timer fields!
	 * They are reset without having the lock.
	 * It is assumed that no code but this will
	 * accesses any field until the ->timeout
	 * callback is called.
	 */
	p->next = NULL;
	p->prev = NULL;
	p->slot = 0;
	(*p->timeout)(p->arg);
    }
}

void erts_bump_timer(erts_short_time_t dt) /* dt is value from do_time */
{
    int i;

    for (i = 0; i < ntiwi; ++i) {
	erts_smp_mtx_lock(&tiwi[i].tiw_lock);
	bump_timer_internal(tiwi+i, dt);
    }
}

Uint
erts_timer_wheel_memory_size(void)
{
    return (Uint) TIW_SIZE * sizeof(ErlTimer*) * ntiwi;
}

/* this routine links the time cells into a free list at the start
   and sets the time queue as empty */
void
erts_init_time(void)
{
    int i, t;
    char buf[21]; /* enough for any 64-bit integer */
    size_t bufsize = sizeof(buf);

    /* system dependent init; must be done before do_time_init()
       if timer thread is enabled */
    itime = erts_init_time_sup();

    if (erts_sys_getenv("ERL_TIMER_WHEEL_SIZE", buf, &bufsize) == 0)
	tiw_size = atoi(buf);
    else
	tiw_size = DEF_TIW_SIZE;
    if (erts_sys_getenv("ERL_TIMER_WHEEL_INSTANCES", buf, &bufsize) == 0)
	ntiwi = atoi(buf);
    else
	ntiwi = DEF_TIW_INSTANCES;

    tiwi = (ErlTiwInstance*) erts_alloc(ERTS_ALC_T_TIMER_WHEEL, ntiwi * sizeof(tiwi[0]));
    for (t = 0; t < ntiwi; ++t) {
	erts_smp_mtx_init_x(&tiwi[t].tiw_lock, "timer_wheel", make_small(t+1));

	tiwi[t].tiw = (struct tiw_head*) erts_alloc(ERTS_ALC_T_TIMER_WHEEL, TIW_SIZE * sizeof(tiwi[0].tiw[0]));
	for(i = 0; i < TIW_SIZE; i++)
	    tiwi[t].tiw[i].head = tiwi[t].tiw[i].tail = NULL;
	tiwi[t].tiw_pos = tiwi[t].tiw_nto = 0;
	tiwi[t].tiw_min_ptr = NULL;
	reset_tiw_min(tiwi+t);
    }

    do_time_init();
}




/*
** Insert a process into the time queue, with a timeout 't'
*/
static void
insert_timer(ErlTiwInstance* tiwi, ErlTimer* p, Uint t)
{
    Uint tm;
    Uint64 ticks;
    ErlTimer* tp;
    ErlTimer* next;

    /* The current slot (tiw_pos) in timing wheel is the next slot to be
     * be processed. Hence no extra time tick is needed.
     *
     * (x + y - 1)/y is precisely the "number of bins" formula.
     */
    ticks = (t + itime - 1) / itime;

    /* 
     * Ticks must be a Uint64, or the addition may overflow here,
     * resulting in an incorrect value for p->count below.
     */
    ticks += do_time_update(); /* Add backlog of unprocessed time */
    
    /* calculate slot */
    tm = (ticks + tiwi->tiw_pos) % TIW_SIZE;
    p->slot = (Uint) tm;
    p->count = tiwi->tiw_count + (Uint) ((ticks + tiwi->tiw_pos) / TIW_SIZE);
  
    /* insert in sorted order */
    if (!tiwi->tiw[tm].head || p->count <= tiwi->tiw[tm].head->count) {
	next = tiwi->tiw[tm].head;
	tp = NULL;
    } else {
	/* scan from tail since new timers are more likely to be after existing timers */
	next = NULL;
	for (tp = tiwi->tiw[tm].tail; tp && p->count < tp->count; next = tp, tp = tp->prev);
    }
    if (tp) {
	p->next = tp->next;
	tp->next = p;
	p->prev = tp;
    } else {
	p->next = next;
	p->prev = NULL;
	tiwi->tiw[tm].head = p;
    }
    if (next) {
	next->prev = p;
    } else {
	tiwi->tiw[tm].tail = p;
    }

    /* insert min time */
    if ((tiwi->tiw_nto == 0) || ((tiwi->tiw_min_ptr != NULL) && (ticks < read_tiw_min(tiwi)))) {
	set_tiw_min(tiwi, ticks);
	tiwi->tiw_min_ptr = p;
    } else if ((tiwi->tiw_min_ptr == p) && (ticks > read_tiw_min(tiwi))) {
	/* some other timer might be 'min' now */
	reset_tiw_min(tiwi);
	tiwi->tiw_min_ptr = NULL;
    }

    tiwi->tiw_nto++;
}

void
erts_set_timer(ErlTimer* p, ErlTimeoutProc timeout, ErlCancelProc cancel,
	      void* arg, Uint t)
{
    int i;

    erts_deliver_time();
    i = erts_get_scheduler_id() % ntiwi;
    erts_smp_mtx_lock(&tiwi[i].tiw_lock);
    if (p->active) { /* XXX assert ? */
	erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
	return;
    }
    p->timeout = timeout;
    p->cancel = cancel;
    p->arg = arg;
    p->active = 1;
    p->instance = i;
    insert_timer(tiwi+i, p, t);
    erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
#if defined(ERTS_SMP)
    if (t <= (Uint) ERTS_SHORT_TIME_T_MAX)
	erts_sys_schedule_interrupt_timed(1, (erts_short_time_t) t);
#endif
}

void
erts_cancel_timer(ErlTimer* p)
{
    int i;

    i = p->instance;
    erts_smp_mtx_lock(&tiwi[i].tiw_lock);
    if (!p->active) { /* allow repeated cancel (drivers) */
	erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
	return;
    }

    /* is it the 'min' timer, remove min */
    if (p == tiwi[i].tiw_min_ptr) {
	tiwi[i].tiw_min_ptr = NULL;
	reset_tiw_min(tiwi+i);
    }

    remove_timer(tiwi+i, p);
    p->slot = p->count = 0;

    if (p->cancel != NULL) {
	erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
	(*p->cancel)(p->arg);
	return;
    }
    erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
}

/*
  Returns the amount of time left in ms until the timer 'p' is triggered.
  0 is returned if 'p' isn't active.
  0 is returned also if the timer is overdue (i.e., would have triggered
  immediately if it hadn't been cancelled).
*/
static ERTS_INLINE Uint
time_left_internal (ErlTiwInstance* tiwi, ErlTimer *p)
{
    Uint left;
    erts_aint_t dt;

    left = (p->count - tiwi->tiw_count)*TIW_SIZE + p->slot - tiwi->tiw_pos;

    dt = do_time_read();
    if (left < dt)
	left = 0;
    else
	left -= dt;
    return left;
}

Uint
erts_time_left(ErlTimer *p)
{
    Uint left;
    int i;

    i = p->instance;
    erts_smp_mtx_lock(&tiwi[i].tiw_lock);

    if (!p->active) {
	erts_smp_mtx_unlock(&tiwi[i].tiw_lock);
	return 0;
    }

    left = time_left_internal(tiwi+i, p);

    erts_smp_mtx_unlock(&tiwi[i].tiw_lock);

    return (Uint) left * itime;
}

#ifdef DEBUG
static void p_slpq_internal (ErlTiwInstance* tiwi)
{
    int i;
    ErlTimer* p;
  
    erts_smp_mtx_lock(&tiwi->tiw_lock);

    /* print the whole wheel, starting at the current position */
    erts_printf("\ntiw_count=%u tiw_pos=%d tiw_nto=%d dt=%u tiw_min_ptr=%p tiw_min=%u\n",
		tiwi->tiw_count, tiwi->tiw_pos, tiwi->tiw_nto, do_time_read(), tiwi->tiw_min_ptr, read_tiw_min(tiwi));
    i = tiwi->tiw_pos;
    do {
	if (tiwi->tiw[i].head != NULL) {
	    erts_printf("%d:\n", i);
	    for(p = tiwi->tiw[i].head; p != NULL; p = p->next) {
		erts_printf(" %p (count %d, slot %d, count_rem = %u, time_left = %u)\n",
			    p, p->count, p->slot, p->count - tiwi->tiw_count, time_left_internal(tiwi, p));
	    }
	}
	if (++i == TIW_SIZE) {
	    i = 0;
	}
    } while (i != tiwi->tiw_pos);

    erts_smp_mtx_unlock(&tiwi->tiw_lock);
}

void erts_p_slpq(void)
{
    int i;

    for (i = 0; i < ntiwi; ++i) {
	p_slpq_internal(tiwi+i);
    }
}
#endif /* DEBUG */
