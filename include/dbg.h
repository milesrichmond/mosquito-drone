#include <stdio.h>

extern void initialise_monitor_handles(void);

void dbg_init(void)
{
    initialise_monitor_handles();
}

void dbg_log(char str[])
{
    printf("%s", str);
}
