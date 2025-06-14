
#define DEBUG_TIME false // 打印时间间隔

#define DEBUG_DURATION 10 // 輸出頻率，建議為10，
#define DEBUG_REMOTOR false // 打印遥控器数据
#define DEBUG_MPU false // 打印陀螺儀的調試信息

int cnt;
unsigned long __lastTime = 0;
unsigned long loopCount = 0;

// 测试数据打印函数 用于打印调试相关数据
void testdataprint()
{
  #if DEBUG_TIME == true
    loopCount++;
    unsigned long currentTime = micros();
    Serial.println(currentTime - __lastTime);
    __lastTime = currentTime;
  #endif

  if (cnt++ > DEBUG_DURATION)
  {
    cnt = 0;
    
    #if DEBUG_MPU == true
      Serial.print(gyroZ);
      Serial.print("\t");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print(gyroY);
      Serial.print("\t");
      Serial.println(ZeparamremoteValue);
    #endif

    #if DEBUG_REMOTOR == true
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        Serial.print(ppmValues[i]);
        if (i< NUM_CHANNELS-1) Serial.print(",");
        else Serial.println();
      }
    #endif
  }
}