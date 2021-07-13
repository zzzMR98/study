#!/usr/bin/python3
from tkinter import *

hit_and_miss_prob = [0.6, 0.4]
prob = 0.5

def bayers_update_callback(state):
    global prob
    if state == 1:
        tem_odds  = (prob*hit_and_miss_prob[0])/(1-hit_and_miss_prob[0]-prob +hit_and_miss_prob[0]*prob)
    else:
        tem_odds  = (prob*hit_and_miss_prob[1])/(1-hit_and_miss_prob[1]-prob +hit_and_miss_prob[1]*prob)
    tem_value = tem_odds/(1+tem_odds)
    if (tem_value>0.9): tem_value = 0.9
    if (tem_value<0.2): tem_value = 0.2
    prob = tem_value
    # print(prob)

def set_current_value():
    prob = float(scl1.get())
    prob_lbl.configure(text=str(prob))
def set_hit_prob():
    hit_and_miss_prob[0] = float(scl2.get())
def set_miss_prob():
    hit_and_miss_prob[1] = float(scl3.get())
def miss_update():
    bayers_update_callback(0)
    prob_lbl.configure(text=str(round(prob, 3)))
def hit_update():
    bayers_update_callback(1)
    prob_lbl.configure(text=str(round(prob, 3)))
def show_prob(value):
    prob_lbl.configure(text=str(round(float(value), 3)))

window = Tk()
window.title("bayers_update_simulation")
window.geometry("350x200")
btn1 = Button(window, text="set current prob", width=15, command=set_current_value)#.pack()
btn1.grid(column=0, row=0)
scl1 = Scale(window,from_=0.,to=1.,resolution=0.1,variable=prob,command=show_prob,orient = HORIZONTAL)#.pack()
scl1.grid(column=1, row=0)
scl1.set(prob)
btn2 = Button(window, text="set hit prob", width=15, command=set_hit_prob)
btn2.grid(column=0, row=1)
scl2 = Scale(window,from_=0.,to=1.,resolution=0.1,variable=hit_and_miss_prob[0],orient = HORIZONTAL)#.pack()
scl2.grid(column=1, row=1)
scl2.set(hit_and_miss_prob[0])
btn3 = Button(window, text="set miss prob", width=15, command=set_miss_prob)
btn3.grid(column=0, row=2)
scl3 = Scale(window,from_=0.,to=1.,resolution=0.1,variable=hit_and_miss_prob[1],orient = HORIZONTAL)#.pack()
scl3.grid(column=1, row=2)
scl3.set(hit_and_miss_prob[1])
miss_btn = Button(window, text="miss", command=miss_update)
miss_btn.grid(column=0, row=3)
prob_lbl = Label(window, text="0.5")#, width=6)
prob_lbl.grid(column=1, row=3)
hit_btn = Button(window, text="hit", command=hit_update)
hit_btn.grid(column=2, row=3)


if __name__ == "__main__":
    window.mainloop()
