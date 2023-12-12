import cv2

# 設定視頻捕捉對象，0 通常是預設的內建網絡攝像頭
cap = cv2.VideoCapture(0)

# 定義編解碼器並創建 VideoWriter 對象
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# 檢查攝像頭是否成功開啟
if not cap.isOpened():
    print("無法開啟攝像頭")
    exit()

while True:
    # 捕捉逐幀影像
    ret, frame = cap.read()

    # 如果正確讀取了影像，ret 是 True
    if not ret:
        print("無法讀取攝像頭影像")
        break

    # 將影像寫入輸出檔案
    out.write(frame)

    # 顯示影像
    cv2.imshow('Frame', frame)

    # 按 'q' 鍵退出循環
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 釋放攝像頭和輸出對象，並關閉所有窗口
cap.release()
out.release()
cv2.destroyAllWindows()