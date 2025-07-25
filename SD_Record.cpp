//メモ：SDの記録でカクツキが起こるのでバッファにためて数十～数百ごとに記録したいかった
//1秒間に約130回書き込みます。バッファはいらないくらい早いかも

#include "FS.h"
#include "SD_Record.h"
#include "SD.h"

void writeFile(fs::FS &fs, const char *path, const char *message) {  //ファイルに上書き
  Serial.printf("ファイルに上書きします: %s\n", path);

  File file = fs.open(path, FILE_WRITE);  //FILE_WRITEで書き込みモード

  if (!file) {
    Serial.println("！ファイルを開くのに失敗しました");
    return;
  }

  if (file.print(message)) {  //引数のmessageを書き込む//書き込み成功すると書き込んだバイト数を返す(０じゃない)
    Serial.println("書き込みに成功しました");
  } else {
    Serial.println("！書き込みに失敗しました");
  }
  
  Serial.println("");
  file.close();
}


void appendFile(fs::FS &fs, const char *path, const char *message) {  //ファイルに追記
  Serial.printf("ファイルに追記します: %s\n", path);

  File file = fs.open(path, FILE_APPEND);  //FILE_WRITEで追記モード

  if (!file) {                         
    Serial.println("！ファイルを開くのに失敗しました");
    return;
  }

  if (file.print(message)) {
    Serial.println("追記に成功しました");
  } else {
    Serial.println("！追記に失敗しました");
  }

  Serial.println("");
  file.close();
}