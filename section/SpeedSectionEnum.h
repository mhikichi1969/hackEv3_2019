#ifndef __SPEED_SECTION_ENUM_H__
#define  __SPEED_SECTION_ENUM_H__
enum class SPEED_SEC_CMD : int
{ 
  LINE_=0x100,
  STRAIGHT_=0x200,
  TURN_=0x300,
  VIRTUAL_=0x400,
  VIRTUAL_LINE_=0x500,
  SET_GOAL_PT_=0x600,
  RESET_LENGTH_=0x700,
  ARM_=0x800,
  PARAM_END = 0xff00
};
#endif
