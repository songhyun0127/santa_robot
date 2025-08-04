from PIL import Image, ImageTk
import tkinter as tk
from ui.style import BUTTON_FONT
from logic.santa_logic import handle_send_gift
#import rclpy
#from std_msgs.msg import String

class GiftUI:
    def __init__(self, root):
        self.root = root
        self.root.title("포장하기")
        self.root.geometry("500x700")
        self.root.resizable(False, False)

        image_path = "santaui/image/santa.jpg"
        img = Image.open(image_path)
        img = img.resize((500, 700))
        self.bg_image = ImageTk.PhotoImage(img)
        bg_label = tk.Label(root, image=self.bg_image)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)

        gift_button = tk.Button(root, text="포장하기", bg="black", fg="white",
                                font=BUTTON_FONT, relief="flat", command=self.show_gift_popup)
        gift_button.pack(side="bottom", fill="x", ipady=15)

    def show_gift_popup(self):
        self.selected_card = "doll"  

        self.doll_img = ImageTk.PhotoImage(Image.open("santaui/image/doll.png").resize((80, 80)))
        self.lego_img = ImageTk.PhotoImage(Image.open("santaui/image/lego.png").resize((80, 80)))

        overlay = tk.Frame(self.root, bg="#000000")
        overlay.place(relx=0, rely=0, relwidth=1, relheight=1)

        popup = tk.Frame(self.root, bg='white', bd=2, relief='ridge')
        popup.place(relx=0.5, rely=0.5, anchor='center', width=350, height=340)

        tk.Label(popup, text="포장할 선물 고르기", font=("맑은 고딕", 13, "bold"), bg="white").pack(pady=(15, 10))

        card_frame = tk.Frame(popup, bg="white")
        card_frame.pack()
        # --- 인형 카드 버튼 ---
        self.doll_btn = tk.Frame(card_frame, bd=2, relief="groove", bg="white",
                                highlightthickness=2, width=150, height=150)
        self.doll_btn.grid(row=0, column=0, padx=10)
        self.doll_btn.propagate(False)
        tk.Label(self.doll_btn, image=self.doll_img, bg="white").pack(pady=(10, 2))
        tk.Label(self.doll_btn, text="인형", font=("맑은 고딕", 12, "bold"), bg="white").pack()
        self.doll_btn.bind("<Button-1>", lambda e: self.select_card("doll"))
        # --- 레고 카드 버튼 ---
        self.lego_btn = tk.Frame(card_frame, bd=2, relief="groove", bg="white",
                                highlightthickness=2, width=150, height=150)
        self.lego_btn.grid(row=0, column=1, padx=10)
        self.lego_btn.propagate(False)

        tk.Label(self.lego_btn, image=self.lego_img, bg="white").pack(pady=(10, 2))
        tk.Label(self.lego_btn, text="레고", font=("맑은 고딕", 12, "bold"), bg="white").pack()
        self.lego_btn.bind("<Button-1>", lambda e: self.select_card("lego"))


        self.select_card("lego")

        tk.Button(popup, text="포장하기", bg="black", fg="white", font=("맑은 고딕", 12),
                  command=lambda: self._confirm_gift(overlay, popup)).pack(pady=(20, 5), ipadx=10, ipady=5)
        tk.Button(popup, text="취소하기", bg="white", fg="gray", relief="flat",
                  command=lambda: self._close_popup(overlay, popup)).pack()

    def select_card(self, card_type):
        self.selected_card = card_type
        if card_type == "doll":
            self.doll_btn.config(highlightbackground="blue")
            self.lego_btn.config(highlightbackground="gray")
        else:
            self.lego_btn.config(highlightbackground="blue")
            self.doll_btn.config(highlightbackground="gray")

    def _confirm_gift(self, overlay, popup):
        handle_send_gift(self.selected_card)
        overlay.destroy()
        popup.destroy()

    def _close_popup(self, overlay, popup):
        overlay.destroy()
        popup.destroy()
    '''
        # 포장하기 버튼 누르면 퍼블리시
    def _confirm_gift(self, overlay, popup):
        # ROS 퍼블리시
        rclpy.init(args=None)
        node = rclpy.create_node("gift_gui_publisher")
        pub = node.create_publisher(String, "/gift/command", 10)

        msg = String()
        msg.data = self.selected_card  
        pub.publish(msg)

        print(f"Published gift command: {msg.data}")
        node.destroy_node()
        rclpy.shutdown()

        overlay.destroy()
        popup.destroy()
'''