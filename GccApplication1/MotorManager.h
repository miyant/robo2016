/*
 * MotorManager.h
 *
 * Created: 2014/10/29 14:46:58
 *  Author: Administrator
 */

#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_
 
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "include/dynamixel.h"

// ------------------ Defined ------------------
#define DEFAULT_BAUDNUM     1 // 1Mbps

//#define DINAMIXEL_AX_12       // Dinamixel AX-12 use
#define LOW_SPEED_MODE          // Low speed mode
//#define HIGH_SPEED_MODE         // High speed mode
//#define MAX_SPEED_MODE          // Max speed mode
//#define TEST_SPEED_MODE        //TEST 20150612

// Motor Settings Address
#define P_CW_ANGLE_LIMIT_L  6
#define P_CW_ANGLE_LIMIT_H  7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_GOAL_SPEED_L      32
#define P_GOAL_SPEED_H      33
#define P_EEP_LOCK          47

#ifdef DINAMIXEL_AX_12
#define RIGHT_MOTOR         2       // Right Motor address
#define LEFT_MOTOR          3       // Left Motor address
#define DELAY_MSEC          10      // Delay time
#define OVER_RUN_TIME       1500    // Over run time
#else // DINAMIXEL_AX_12
#define RIGHT_MOTOR         30      // Right Motor address
#define LEFT_MOTOR          31      // Left Motor address
#define DELAY_MSEC          1       // Delay time
#define OVER_RUN_TIME       500     // Over run time
#endif // DINAMIXEL_AX_12

#define COVER_MOTOR			18		// Cover Motor address

// Motor Speed Value
#ifdef DINAMIXEL_AX_12
#define P_CW_SPEED_NOMAL    1023
#define P_CCW_SPEED_NOMAL   2047
#define P_CW_SPEED_TURN     920
#define P_CCW_SPEED_TURN    1943
#define P_CW_SPEED_TURN_2   818
#define P_CCW_SPEED_TURN_2  1841
#else // DINAMIXEL_AX_12
// DINAMIXEL_AX_12W
#if defined (LOW_SPEED_MODE)
// LOW_SPEED_MODE
#define P_CW_SPEED_NOMAL    609
#define P_CCW_SPEED_NOMAL   1632
#define P_CW_SPEED_TURN     509
#define P_CCW_SPEED_TURN    1532
#define P_CW_SPEED_TURN_2   409
#define P_CCW_SPEED_TURN_2  1432
#elif defined (HIGH_SPEED_MODE)
// HIGH_SPEED_MODE
#define P_CW_SPEED_NOMAL    809
#define P_CCW_SPEED_NOMAL   1832
#define P_CW_SPEED_TURN     709
#define P_CCW_SPEED_TURN    1732
#define P_CW_SPEED_TURN_2   609
#define P_CCW_SPEED_TURN_2  1632
#elif defined (TEST_SPEED_MODE)
// TEST_SPEED_MODE
#define P_CW_SPEED_NOMAL    200
#define P_CCW_SPEED_NOMAL   1223
#define P_CW_SPEED_TURN     1203
#define P_CCW_SPEED_TURN    180
#define P_CW_SPEED_TURN_2   1153
#define P_CCW_SPEED_TURN_2  150
#else
// MAX_SPEED_MODE
#define P_CW_SPEED_NOMAL    1023
#define P_CCW_SPEED_NOMAL   2047
#define P_CW_SPEED_TURN     920
#define P_CCW_SPEED_TURN    1943
#define P_CW_SPEED_TURN_2   818
#define P_CCW_SPEED_TURN_2  1841
#endif // LOW_SPEED_MODE
#endif // DINAMIXEL_AX_12

// Max count in Test Mode
#define MAX_COUNT 28

// Move Type
#define MOVE_SELECTION_TYPE_START       1000
#define MOVE_SELECTION_TYPE_STOP        1001
#define MOVE_SELECTION_TYPE_STRAIGHT    1002
#define MOVE_SELECTION_TYPE_RIGHTSIFT_1 1003
#define MOVE_SELECTION_TYPE_RIGHTSIFT_2 1004
#define MOVE_SELECTION_TYPE_LEFTSIFT_1  1005
#define MOVE_SELECTION_TYPE_LEFTSIFT_2  1006
#define MOVE_SELECTION_TYPE_BACK        1007
#define MOVE_SELECTION_TYPE_RIGHTTURN   1008
#define MOVE_SELECTION_TYPE_LEFTTURN    1009
#define MOVE_SELECTION_TYPE_RIGHTTURN_2 1010
#define MOVE_SELECTION_TYPE_LEFTTURN_2  1011
#define MOVE_SELECTION_TYPE_SEARCH      1012
#define MOVE_SELECTION_TYPE_STRAIGHT_2	1013
#define MOVE_SELECTION_TYPE_SLOWBACK    1014
#define MOVE_SELECTION_TYPE_RIGHTTURN_3 1015
#define MOVE_SELECTION_TYPE_LEFTTURN_3  1016

// Special Move
#define MOVE_SELECTION_TYPE_S_MOVE_1    1100
#define MOVE_SELECTION_TYPE_S_MOVE_2    1101
#define MOVE_SELECTION_TYPE_S_MOVE_3    1102
#define MOVE_SELECTION_TYPE_S_MOVE_4    1104
#define MOVE_SELECTION_TYPE_S_MOVE_5    1105
#define MOVE_SELECTION_TYPE_S_MOVE_6    1106
#define MOVE_SELECTION_TYPE_S_MOVE_7    1107
#define MOVE_SELECTION_TYPE_S_MOVE_8    1108
#define MOVE_SELECTION_TYPE_S_MOVE_9    1109	// Fixed Mode
#define MOVE_SELECTION_TYPE_S_MOVE_10   1110	// Test Mode

// ------------------ Method Definition ------------------
void MotorInit(void);
void MotorControl(int id, int power);
void Execute(int type);
void setParamMoveAction(int right, int left);
void StopMove(void);
void StraightMove(void);
void StraightLowMove(void);
void StraightMoveRightShift(void);
void StraightMoveLeftShift(void);
void StraightMoveRightShift2(void);
void StraightMoveLeftShift2(void);
void TurnMoveRight(void);
void TurnMoveLeft(void);
void TurnLowMoveRight(void);
void TurnLowMoveLeft(void);
void BackMove(void);
void TestMode(void);
void PrintErrorCode(void);
void PrintCommStatus(int CommStatus);

void AdjustSpeed(int targetSpeedL, int targetSpeedR);
int GetCurrentSpeed(int id);

int mCount;

#endif // MOTOR_MANAGER_H_