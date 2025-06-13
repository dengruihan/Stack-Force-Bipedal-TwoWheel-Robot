// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

#include <WiFi.h>
#include <SPIFFS.h>

/***********html&javascript网页构建**********/
// 从文件系统读取HTML内容的函数
String loadHTMLFromFile() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS初始化失败");
    return "";
  }
  
  File file = SPIFFS.open("/robot_web_control.html", "r");
  if (!file) {
    Serial.println("无法打开HTML文件: /robot_web_control.html");
    return "";
  }
  
  String htmlContent = file.readString();
  file.close();
  Serial.println("成功从文件系统加载HTML内容");
  return htmlContent;
}

// 获取HTML内容的函数（带缓存）
String getBasicWebContent() {
  static String htmlContent = "";
  static bool loaded = false;
  
  if (!loaded) {
    htmlContent = loadHTMLFromFile();
    loaded = true;
  }
  return htmlContent;
}

// 为了向后兼容，保留原来的变量名
// 使用时调用 getBasicWebContent() 而不是直接使用 basic_web
#define basic_web getBasicWebContent().c_str()
