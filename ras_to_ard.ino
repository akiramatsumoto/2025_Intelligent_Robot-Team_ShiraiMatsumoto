// ras_to_ard.ino

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Leonardo/Micro などではシリアル接続完了待ち
  }
  Serial.println("Arduino Ready");
}

void loop() {
  // 受信バッファが溜まっていれば1行読み込み
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();  // 前後の改行・空白を削除

    if (line.length() == 0) {
      return;
    }

    // カンマで分割
    int idx1 = line.indexOf(',');
    int idx2 = line.indexOf(',', idx1 + 1);

    if (idx1 > 0 && idx2 > idx1) {
      String color = line.substring(0, idx1);
      String area  = line.substring(idx1 + 1, idx2);
      String angle = line.substring(idx2 + 1);

      // シリアルモニタ出力
      Serial.print("Received -> Color: ");
      Serial.print(color);
      Serial.print(" | Area: ");
      Serial.print(area);
      Serial.print(" px^2 | Angle: ");
      Serial.print(angle);
      Serial.println(" deg");
    } else {
      // フォーマットエラー
      Serial.print("Bad data: ");
      Serial.println(line);
    }
  }
}

