import can
import tkinter as tk
from tkinter import Entry, Label, Button, Text, Checkbutton

class CANMessageSender:
    def __init__(self, master):
        self.master = master
        master.title("Custom CAN Message Sender")

        self.label_interface = Label(master, text="Interface:")
        self.label_id = Label(master, text="CAN ID:")
        self.label_payload = Label(master, text="Payload:")
        self.entry_interface = Entry(master)
        self.entry_id = Entry(master, validate='key', validatecommand=(master.register(self.validate_hex), '%P'))
        self.entry_payload = [Entry(master, width=3, validate='key', validatecommand=(master.register(self.validate_hex), '%P')) for _ in range(8)]

        self.label_interface.grid(row=0, column=0, padx=10, pady=10)
        self.label_id.grid(row=0, column=2, padx=2, pady=2)
        self.label_payload.grid(row=0, column=4, padx=10, pady=10)
        self.entry_interface.grid(row=0, column=1, padx=10, pady=10)
        self.entry_id.grid(row=0, column=3, padx=2, pady=2)
        for i, entry in enumerate(self.entry_payload):
            entry.grid(row=0, column=5+i, padx=2, pady=10)

        self.text_output = Text(master, height=10, width=60)
        self.text_output.grid(row=1, columnspan=12, padx=10, pady=10)

        self.send_button = Button(master, text="Send Message", command=self.send_message)
        self.send_button.grid(row=2, columnspan=12, pady=10)

        self.loop_button = Checkbutton(master, text="Loop", command=self.toggle_loop)
        self.loop_button.grid(row=2, column=1, pady=10)

        self.loop_task_id = None
        self.loop_enabled = False
        

    def send_message(self):
        try:
            can_interface = self.entry_interface.get()
            can_id_str = self.entry_id.get()
            can_id = int(can_id_str, 16)
            payload = [int(entry.get(), 16) for entry in self.entry_payload]
            message_str = f"{can_interface} {can_id:X} {' '.join(map(lambda x: f'{x:02X}', payload))}"
            send_custom_message_from_string(message_str)

            self.text_output.insert(tk.END, message_str + "\n")
            self.text_output.yview(tk.END)
        except ValueError as e:
            print(f"Error: {e}")
    
    def toggle_loop(self):
        self.loop_enabled = not self.loop_enabled

        if self.loop_enabled:
            self.start_loop()
        elif self.loop_task_id:
            self.master.after_cancel(self.loop_task_id)
            self.loop_task_id = None

    def start_loop(self):
        if self.loop_task_id:
            self.master.after_cancel(self.loop_task_id)

        if self.loop_enabled:
            self.loop_task_id = self.master.after(1000, self.send_loop_message)

    def send_loop_message(self):
        self.send_message()
        if self.loop_enabled:
            self.loop_task_id = self.master.after(1000, self.send_loop_message)

    def validate_hex(self, value):
        return all(c in '0123456789ABCDEFabcdef' for c in value)

def send_custom_message_from_string(message_str):
    parts = message_str.split()
    if len(parts) < 3:
        raise ValueError("Invalid message format")

    can_interface = parts[0]
    can_id = int(parts[1], 16)
    data = [int(byte, 16) for byte in parts[2:]]

    bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
    try:
        message = can.Message(arbitration_id=can_id, is_extended_id=False, data=data)
        bus.send(message)
    finally:
        bus.shutdown()

if __name__ == "__main__":
    root = tk.Tk()
    app = CANMessageSender(root)
    root.mainloop()
