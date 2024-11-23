/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    RJ25AdapterTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/02
 * @brief   Description: this file is sample code for RJ25 adapter module
 *
 * Function List:
 * 1. bool MeRGBLed::dWrite1(bool value)
 * 2. bool MeRGBLed::dRead2()
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/02    1.0.0          rebuild the old lib
 * </pre>
 */

#include "MeAuriga.h"

MePort input(PORT_9);


int val;
void setup()
{
  Serial.begin(9600);
}

void loop()
{
  val = input.();   /* read SLOT1 level */
  if (val != 0)
  {
    Serial.print("val=");
    Serial.println(val);
  }
}

