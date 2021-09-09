
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#define SEC_IN_NSEC_PT 1000000000

int CreatePosixTask(char *TaskName, int Priority, int StackSizeInKo, pthread_t *posix_thread, void *(*pTaskFunction)(void *))
{   
    pthread_attr_t ThreadAttributes;
    int err = pthread_attr_init(&ThreadAttributes);
    if (err)
    {
        printf("pthread attr_init() failed for thread '%s' with err=%d\n", TaskName, err);
        return -10;
    }
    err = pthread_attr_setinheritsched(&ThreadAttributes, PTHREAD_EXPLICIT_SCHED);
    if (err)
    {
        printf("pthread set explicit sched failed for thread '%s' with err=%d\n", TaskName, err);
        return -11;
    }
    err = pthread_attr_setdetachstate(&ThreadAttributes, PTHREAD_CREATE_JOINABLE /*PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE*/);
    if (err)
    {
        printf("pthread set detach state failed for thread '%s' with err=%d\n", TaskName, err);
        return -12;
    }
    err = pthread_attr_setschedpolicy(&ThreadAttributes, SCHED_FIFO);
    if (err)
    {
        printf("pthread set scheduling policy failed for thread '%s' with err=%d\n", TaskName, err);
        return -13;
    }
    struct sched_param paramA = {.sched_priority = Priority};
    err = pthread_attr_setschedparam(&ThreadAttributes, &paramA);
    if (err)
    {
        printf("pthread set priority failed for thread '%s' with err=%d\n", TaskName, err);
        return -14;
    }
    // if (StackSizeInKo > 0)
    // {
    //     err = pthread_attr_setstacksize(&ThreadAttributes, StackSizeInKo * 1024);
    //     if (err)
    //     {
    //         printf("pthread set stack size failed for thread '%s' with err=%d\n", TaskName, err);
    //         return -15;
    //     }
    // }   

    err = pthread_create(posix_thread, &ThreadAttributes, (void *(*)(void *))pTaskFunction, (void *)NULL);

    if (err)
    {
        printf("Failed to create thread '%s' with err=%d !!!!!\n", TaskName, err);
        return -1;
    }
    else
    {
        pthread_attr_destroy(&ThreadAttributes);
        // err = pthread_setname_np(*posix_thread, TaskName);
        // if (err)
        // {
        //     printf("pthread set name failed for thread '%s', err=%d\n", TaskName, err);
        //     return -40;
        // }
        // printf("Created thread '%s'\n", TaskName);
        return 0;
    }
}

void WaitPeriodicWithNanocleep(struct timespec *ts, int64_t ns, int64_t toff)
{
    ts->tv_nsec += ns + toff;
    if (ts->tv_nsec >= SEC_IN_NSEC_PT)
    {
        ts->tv_nsec -= SEC_IN_NSEC_PT;
        ts->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
}

int64_t getdiff(struct timespec *t_expect_, struct timespec *t_now)
{
    return (t_now->tv_sec - t_expect_->tv_sec) * SEC_IN_NSEC_PT + (t_now->tv_nsec - t_expect_->tv_nsec);
}