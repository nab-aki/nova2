// Wi-Fi の設定の見本。このファイルを wifi_secrets.h という名前でコピーして、値を書く
// wifi_secrets.h は .gitignore 済み（リポジトリに入れない）。ESP32 がつなげるのは 2.4GHz だけ
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "ここにSSID"
#define WIFI_PASSWORD "ここにパスワード"

// IP の固定（任意）。下の4行を全部有効にしたときだけ固定する。書かなければ DHCP
// 【注意】IP は、ルーターの DHCP の範囲の外の番号を選ぶ（範囲の中だと、ほかの機器と重なることがある）
// #define WIFI_STATIC_IP      "192.168.1.200"
// #define WIFI_STATIC_GATEWAY "192.168.1.1"
// #define WIFI_STATIC_SUBNET  "255.255.255.0"
// #define WIFI_STATIC_DNS     "192.168.1.1"

#endif  // WIFI_SECRETS_H
