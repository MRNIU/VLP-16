
// This file is a part of MRNIU/LUTF
// (https://github.com/MRNIU/LUTF).
//
// test.h for MRNIU/LUTF.

#ifndef _TEST_H_
#define _TEST_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*UnitTestFunction)(void);

void run_tests(UnitTestFunction *tests);

// FIFO
int fifo(void);
// TIME
int time_(void);
// 任务管理
int test_manage(void);

#ifdef __cplusplus
}
#endif

#endif /* _TEST_H_ */
