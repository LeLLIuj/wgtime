#ifndef __SYS_TICK_H_
#define __SYS_TICK_H_

void initSysTick(void);
inline uint32 getSysTickCount(void);
inline uint32 getMicroSysTickCount(void);

#endif //__SYS_TICK_H_