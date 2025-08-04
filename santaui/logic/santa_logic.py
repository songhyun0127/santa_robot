from tkinter import messagebox

def handle_send_gift(card_type):
    if card_type == "doll":
        messagebox.showinfo("포장 완료", "인형이 포장되었습니다!")
    elif card_type == "lego":
        messagebox.showinfo("포장 완료", "레고가 포장되었습니다!")
