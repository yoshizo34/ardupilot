from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil

# 機体に接続
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

# コマンドオブジェクトの取得
cmds = vehicle.commands

# # ミッションのダウンロード実行
# cmds.download()
# cmds.wait_ready()
# print("mission download Done.")

# # ミッションのクリアとアップロード
# cmds.clear()
# cmds.upload()
# print("mission clear and upload Done.")

# ミッションの生成と追加(p.80)
# cmds.download()                                     # ダウンロード
# cmds.wait_ready()                                   # コマンド完了待ち
# print("download done.")

# # コマンド定義(テイクオフのミッションを定義)
# cmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
# cmds.add( cmd )                                     # コマンドの追加
# print("add command done.")

# cmds.upload()                                       # アップロード
# cmds.wait_ready()
# print("mission upload done.")

# ミッションの変更(p.81)
cmds.download()                                         # ダウンロード実行
cmds.wait_ready()
print("download done.")

# 編集用に別のリストにミッションを保持
missionList = []
for cmd in cmds:
    missionList.append( cmd )
print("mission backup done.")

# ミッションの変更２(p.82)
# 最初のミッションをテイクオフに変更
missionList[ 0 ].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

cmds.clear()                                            # 現在のミッションをクリア

# 変更したミッションをアップロード
for cmd in missionList:
    cmds.add( cmd )
    cmds.upload()

vehicle.mode = VehicleMode("AUTO")
