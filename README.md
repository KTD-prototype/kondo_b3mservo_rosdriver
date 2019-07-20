# kondo-b3mservo-rosdriver
<!-- TOC depthFrom:1 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [kondo-b3mservo-rosdriver](#kondo-b3mservo-rosdriver)
	- [概要](#概要)
	- [パッケージ説明](#説明)
	- [デモ動画](#動画)
	- [動作確認済みの環境](#動作確認済環境)
	- [インストール](#)
	- [使用する前に](#使用前)
	- [使い方](#使方)
		- [ノードの構成](#構成)
		- [ターミナルから直接ROSメッセージを送って制御する場合](#直接ros送制御場合)
		- [ジョイスティックによる制御](#制御)
		- [サーボの情報を取得する](#情報取得)

<!-- /TOC -->

## 概要
[README file in English is available](https://github.com/KTD-prototype/kondo_b3mservo_rosdriver/blob/master/README_EN.md)<br>
近藤科学のサーボモータ（B3Mシリーズ）をROSで動かすためのパッケージです。<br>
日本語のコメントが入ったバージョンも準備中です！
<br>
<br>
<br>

## パッケージ説明
近藤科学のサーボモータ（B3Mシリーズ）をROSで動かすためのパッケージです。<br>
ROSメッセージを介して、サーボに指令値を送信し、位置制御、速度制御、トルク制御ができます。<br>
位置、速度、トルクの各制御モードは、立ち上げるノードを選ぶことで切り替えます。<br>
<br>
近藤科学のB3Mシリーズサーボは予めIDが振られていますが、このパッケージでは、接続されたサーボのIDを自動検出します。<br>
指令値は対象のサーボIDが小さい順に並んだ配列として送信します。<br>
<br>
<br>
<br>

## デモ動画
コンソールからのROSメッセージパブリッシュによる位置制御、ゲームパッドによるトルク制御のデモ<br>
![result](https://github.com/KTD-prototype/kondo_b3mservo_rosdriver/blob/media/sample.gif?raw=true)
<br>
<br>
<br>

## 動作確認済みの環境
以下の環境で動作確認しています:
  * Ubuntu16.04
  * python2.7.12
  * ROS kinetic kame
* サーボ : [B3M-SC-1170-A](https://kondo-robot.com/product/03092)
* PCとサーボを接続するUSB-シリアルI/F : [RS485-USB adapter](https://kondo-robot.com/product/02133)
* 電源 : 3セル LiPo バッテリ
* ゲームパッド：[ロジクールF710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html)

近藤科学のシリアル変換アダプタをUbuntuで使用するためには、事前にドライバのインストールが必要です。[やりかたはこちら](https://kondo-robot.com/faq/usb_adapter_for_linux_2019)<br>
また、シリアル変換アダプタのUbuntu上での設定を何点か変更しています。
* デバイス名を***Kondo_USB-RS485_converter***に固定
* シリアル通信のレイテンシタイマを1msに固定（デフォルトは16msのため、サーボへのコマンド送信ごとに16msかかり、制御周期が上げられない）<br>
参考リンク：[udev_rulesでデバイス名固定とファイル書き込み](https://woodencaliper.hatenablog.com/entry/2018/06/30/175622)<br>
今回は[こちらのファイル](https://github.com/KTD-prototype/kondo_b3mservo_rosdriver/blob/master/etc/99-serial.rules)を***/etc/udev/rules.d/***に配置して設定しました。<br>
最後に以下のコマンドでルールの再読み込みが必要です。<br>
`   $ sudo udevadm control --reload-rules`<br>
<br>
<br>
<br>

## インストール(catkin_wsという名前のディレクトリで作業している場合)
`   $ cd ~/catkin_ws/src`<br>
`   $ git clone git@github.com:KTD-prototype/kondo_b3mservo_rosdriver.git`<br>
`   $ cd ~/catkin_ws`<br>
`   $ catkin_make`
<br>
<br>
<br>

## 使い方
### ディレクトリの構成
ディレクトリは以下のようなファイル構成となっています。
  ***scripts***　
  * ***generate_command_autodetect_joy.py***  : ゲームコントローラ（ジョイスティック）の情報を受け取り、サーボ指令値を生成・パブリッシュするノード
  * ***Kondo_B3M_functions.py***  :  サーボへのコマンド関数の集まり
  * ***position_control.py***  : 位置（角度）制御用のノード
  * ***torque_control.py***  : トルク制御用のノード
  * ***velocity_control.py*** : 速度制御用のノード

***config***　
  * ***change_config.py***  : サーボの設定値を変更するpythonスクリプト
  * ***config_function.py***  :  サーボの設定値変更のためのコマンド関数の集まり
  * ***show_config.py***  : 接続されているサーボの設定値をコンソールに表示するスクリプト

***launch***　
  * ***position_control_by_torque_sample.launch***  : ジョイスティックの右スティック（上下）の値に応じたトルク制御（位置フィードバック）を行うlaunchファイル
  * ***position_control_sample.launch***  :  ジョイスティックの左スティック（左右）の値に応じた位置制御を行うlaunchファイル
  * ***torque_control_sample.launch***  : ジョイスティックの左スティック（上下）の値に応じたトルク制御を行うlaunchファイル
  * ***velocity_control_sample.launch***  : ジョイスティックの右スティック（左右）の値に応じた速度制御を行うlaunchファイル

<br>
それぞれのノード（位置、トルク、速度制御ノード）は自動でUSBｰシリアルI/Fが接続されたポートをスキャンして、接続されているサーボの個数とIDを検出します。<br>
サーボへの指令値やサーボからの情報はサーボの個数だけ、IDが小さい順に並んだリストとしてやりとりします。<br>
指令値の単位は　位置：[*0.01°]、速度：[*0.01°/秒]、トルク：[mNm]です。<br>
たとえば位置指令値:[4500]と送れば、サーボは45°の位置に動きます。IDが1, 2のサーボを接続して[4500, -3000]と指令値を送れば、ID1のサーボが45°、ID2のサーボが-30°の位置に動きます。<br>
指令値を送ると、サーボはその時のエンコーダ値等の情報を返してきますので、ROSメッセージの形で読み取り可能です。
<br>
<br>

### ターミナルから直接ROSメッセージを送って制御する場合
最もシンプルなやり方です。
<br>
`		$ roscore`
<br>新しいターミナルを開いて、以下のようにノードを立ち上げます<br>
`		（位置制御の場合）`
`		$ python position_control.py`
`		`
`		（トルク制御の場合）`
`		$ python torque_control.py`
`		`
`		（速度制御の場合）`
`		$ python velocity_control.py`  

そのままだと、以下のようなエラーを吐くと思われます（申し訳ありません）<br>
 ***serial.serialutil.SerialException: [Errno 2] could not open port /dev/Kondo_USB-RS485_converter: [Errno 2] No such file or directory: '/dev/Kondo_USB-RS485_converter'***
<br>
<br>
解決には２通りの方法があって : <br>
  * コード中のデバイス名を変更する : ***position_control.py***および***Kondo_B3M_functions.py***の中にある ***/dev/Kondo_USB-RS485_converter***　を例えば ***/dev/ttyUSB0***　のような、お手持ちのUSB/シリアルI/Fのデバイス名に変更する
  * シンボリックリンクを固定することで、デバイス名を***dev/Kondo_USB-RS485_converter***に変更・固定する（[参考](https://woodencaliper.hatenablog.com/entry/2018/06/30/175622)）
<br>
前者のほうが手っ取り早いですが、PCの再起動やデバイスの再起動のたびにデバイス名が変わる可能性があります。
いずれかの対処をしたら、再度ノードを立ち上げます。
<br>
<br>

例（位置制御の場合）<br>
`		$ python position_control.py`

ノードが立ち上がり、自動で接続されたサーボの個数とIDを検出します。
ノードが立ち上がったら、ターミナルからメッセージをパブリッシュすることで制御できます。
<br>

メッセージ例(位置制御、速度制御、トルク制御　＠サーボ２個の場合)<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_position:[1000, 1000]}"`<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_velocity:[1000, 1000]}"`<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_torque:[500, 500]}"`<br>

送るメッセージの種類は、位置／速度／トルクのどの制御をしているかに依存します。

<br>

### ジョイスティックによる制御
ジョイスティック（ゲームコントローラ）による制御には、Launchファイルを使うと便利です。（事前にジョイスティックのROSのパッケージをインストールする必要あり）
<br>

`		$ roslaunch kondo_b3mservo_rosdver position_control_sample.launch`<br>
`		$ roslaunch kondo_b3mservo_rosdver velocity_control_sample.launch`<br>
`		$ roslaunch kondo_b3mservo_rosdver torque_control_sample.launch`<br>

ジョイスティックのコマンド対応は以下です（指令値を送る前に、どこかのボタンを押す必要があるかもしれません）:
 * 位置制御 : 左スティックの左右方向
 * 速度制御 : 右スティックの左右方向
 * トルク制御 : 左スティックの上下
<br>


### サーボの情報を取得する
サーボの情報を取得する場合、以下のようにメッセージをサブスクライブできます:
`		$ rostopic echo /multi_servo_info`
<br>
今のところ、見られる情報は以下の四種類です。
 * エンコーダのカウント値[count]
 * 電源電圧[mV]
 * サーボ回転速度[*0.01deg/sec]
 * サーボ電流値[mA]
