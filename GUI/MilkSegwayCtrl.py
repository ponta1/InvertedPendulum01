# -*- coding:utf-8 -*-

# MilkSegwayCtrl
#
# pyserialのインストール方法
#   pip install pyserial

import os
import sys
import time
import tkinter
from tkinter import messagebox
import serial

import MilkSegwayCtrl_setting

COM_PORT = MilkSegwayCtrl_setting.COM_PORT

# データファイル名
FILE = os.path.dirname(__file__) + "\\MilkSegway.txt"

TIMER_MS = 10 # タイマーの間隔 0.01秒

timer_count = 0
serial_rcv = ""     # シリアルで受信したデータ

# タイマー処理
def timer():
    global TIMER_MS
    global ser
    global root
    global timer_count
    global serial_rcv

    timer_count = timer_count + 1

    # シリアルの受信
    while ser.in_waiting > 0:
        rcv = ser.read(1)
        if rcv != b'\r' and rcv != b'\n':
            serial_rcv = serial_rcv + rcv.decode('utf-8')
        if rcv == b'\n':
            print("RECV:" + serial_rcv)
            # Arduino側起動後は自動で送信
            if serial_rcv == "MilkSegway Start":
                send()
            serial_rcv = ""
            break

    # 次のタイマーをセット
    root.after(TIMER_MS, timer)

# COM送信処理
def send():
    global ser
    global ar_param_var
    
    send = u"P"
    for i in range(len(ar_param_var)):
        param = ar_param_var[i].get()
        f = float(param)
        sign = "+"
        if f < 0:
            sign = "-"
            f = -f
        x = int(f)
        y = int((f - x) * 100 + 0.5)
        s = sign + "{:0=2}".format(x) + "{:0=2}".format(y)
        send = send + s + " "
        #if i != len(ar_param_var) - 1:
        #    send = send + " "
    send = send + u"E"

    print("SEND:" + send)

    ser.write(send.encode('utf-8'))


# 「送信」ボタンが押された時
def click_send():
    send()

# 「リセット」ボタンが押された時
def click_reset():
    global ser
    
    send = u"R"
    for i in range(48):
        send = send + u" "
    send = send + u"E"

    print("SEND:" + send)
    ser.write(send.encode('utf-8'))

# 「パラメータ」テキストボックス変更
def change_param(i, val, cause):
    if cause == "forced":
        return True

    # 数値チェック
    try:
        v = float(val)  # 文字列を実際にint関数で変換してみる
    except ValueError:
        messagebox.showinfo(u"エラー", u"数値以外が指定されています")
        return False    

    if v < -100 or 100 < v:
        messagebox.showinfo(u"エラー", u"整数部2桁以内です")
        return False    

    return True

# 保存ボタンクリック
def click_save():
    global ar_param_var

    f = open(FILE, "w")

    param = ""
    for i in range(len(ar_param_var)):
        param = param + ar_param_var[i].get()
        if i != len(ar_param_var) - 1:
           param = param + ","

    f.write(param + "\n")
    f.close()

# ファイルロード
def load():
    global ar_param_var

    try:
        f = open(FILE, "r")
        data = f.readline()
    except IOError:
        print("データファイルが開けません " + FILE)
        return
    f.close()

    params = data.split(',')

    for i in range(len(params)):
        ar_param_var[i].set(params[i])

# ======================
# メイン
# ======================
if __name__ == "__main__":

    # ウィンドウ初期化
    root = tkinter.Tk()
    root.title(u"MilkSegway パラメータ設定")

    root.geometry(str(300) + "x" + str(350))   # ウインドウサイズを指定

    ar_k_name = [u'-', u'角速度', u'角度', u'角度の積算', u'-', u'速度', u'-', u'全体']
    ar_k_var = [u'0', u'1.0', u'0.11', u'0.2', u'0', u'2', u'0', u'1']

    # パラメータのテキストボックス
    ar_param_var = []
    ar_param = []
    tcl_change_param = root.register(change_param)
    for i in range(len(ar_k_name)):
        # パラメータ名の表示
        label = tkinter.Label(
            root,
            text=ar_k_name[i]
        )
        label.place(x = 5, y = 30 + i * 40)

        # 値のテキストボックス
        ar_param_var.append(tkinter.StringVar())
        ar_param_var[i].set(ar_k_var[i])
        ar_param.append(
            tkinter.Entry(root, textvariable=ar_param_var[i], validate="focusout", vcmd=(tcl_change_param, i, '%s', '%V'), width=6)
        )
        ar_param[i].place(x = 120, y = 35 + i * 40)

    # -------------------------------------

    X = 200    # コマンドエリア

    # 送信ボタン
    button_send = tkinter.Button(root, text = u"送信", command=click_send, width = 10)
    button_send.place(x = X, y = 20)

    # リセットボタン
    button_reset = tkinter.Button(root, text = u"角度リセット", command=click_reset, width = 10)
    button_reset.place(x = X, y = 60)

    # 保存ボタン
    button_save = tkinter.Button(root, text = u"保存", command=click_save, width = 10)
    button_save.place(x = X, y = 100)

    # -------------------------------------

    try:
        #ser = serial.Serial(COM_PORT, 9600)
        ser = serial.Serial(COM_PORT, 57600)
        #ser = serial.Serial(COM_PORT, 115200)
    except IOError:
        print("COMポートが開けません:" + COM_PORT)
        messagebox.showinfo(u"エラー", u"COMポートが開けません:" + COM_PORT)
        sys.exit()

    load()

    # タイマー開始
    root.after(TIMER_MS, timer)

    #ser.close()
    root.mainloop()
