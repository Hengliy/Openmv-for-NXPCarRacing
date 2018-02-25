import sensor, image, time , math ,time
from pyb import UART
#car_tail = (1,100,-30,20,55, 99) # 尾（黄色）
#car_tail = (1,100,8,27,-1,30 ) # 尾（棕色）
#car_tail = (1,100,29,51,-50,1 ) # 尾（紫色）
#car_tail   = (1,100,23,70,18,33 )(1) # 尾（预计橙色）
#car_tail   = (1,100,20,70,68,90 ) #(0) 尾（预计橙色
#car_tail   = (1,100,20,70,60,90 ) # 尾（预
car_tail   = (1,100,20,70,45,90 )#(2)
car_head =  (1,100,-71,-30,30,80) # 头（绿色）
#car_head =  (1,100,-71,-30,40,80) # 头（亮绿色）
target = (1,100,28,90,-60,20 ) #原始信标灯
#target = (1,100,28,90,10,20 )
target_miss =(35,100,-16,15,-10,44)
#target_miss =(1,100,-22,61,-48,51)

sensor.set_colorbar(True)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.CIF)
sensor.set_windowing((240,240)) # 240x240 center pixels of VGA
sensor.skip_frames(2) #采取每秒40帧
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking ，必须关闭用于颜色跟踪
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
#只有具有比“pixel_threshold”更多的像素和比“area_threshold”更多的区域的斑点
#由下面的“find_blob”返回。 更改“像素阈值”和“区域阈值”
#相机分辨率。 “merge = True”合并图像中的所有重叠的斑点
car = 0
angle_float = 0   #用于记录角度大小
angle_char_send = 0   #用于转换角度大小和A,B,C,D等级关系
x0 = 0  #用来记录车尾位置
y0 = 0
x1 = 0  #用来记录车头位置
y1 = 1  #19
x2 = 0  #用来记录信标位置
y2 = 1
m=0
w = 0
distance = 2  #车灯到车长和车长的倍数
car_long = 0
tail_lamp_long = 0
uart = UART(3, 115200)
flag=1
miss_flag=0
temp_x=0
temp_y=0
count = 0
angle_min = 78
angle_max = 102
lamp_count = 0
R = 16
change_x2 = 0
change_y2 = 0
big_flag = 0
temp_m=1000
speed = '-'
target_miss_list=[]
car_m = 0

##############################################

R_tail = 40 #以绿色为基准，找黄色
yellow_line_not = 80 #排除黄色线避障干扰
miss_max = 45 #避障最大面积
miss_min = 1  #避障最小面积
#还有调视野

##############################################


speed_temp = '+'
speed_flag = 0
speed_flag_count = 0

pre_middle_lamp_long=0 #记录前一次的车灯距离
pre_speed='+'#记录前一次的加减速情况
death_count=0#记录减速区内运行的时间，时间过长判断为进入死区
miss_long=30#第一次减速距离
#**************************************************************************************************#
#                                           主循环                                                  #
#**************************************************************************************************#
while(True):
    R = 16
    angle_int = 0
    big_flag = 0
    clock.tick()
    img = sensor.snapshot()

    #用来定义全局变量blob
    blob=[]
    #记录像素点
    #循环查找符合车尾的blobs，并找出像素最多的赋值给blob
    #***************************************车头****************************************************#
    m1 = 0
    for blobs in img.find_blobs([car_head], pixels_threshold=1, area_threshold=1, merge=True):
        if(m1<blobs.pixels()):
            blob = blobs
            m1 = blobs.pixels()
    if(m1>0):
        img.draw_rectangle(blob.rect())
        x1 = blob.cx()
        y1 = blob.cy()
        img.draw_cross(blob.cx(), blob.cy()) #画十字
    img.draw_circle(int(x1), int(y1),int(R_tail))
    #***************************************车尾***************************************************#
    m1 = 0
    for blobs in img.find_blobs([car_tail], pixels_threshold=1, area_threshold=1, merge=True):
        if m1<blobs.pixels() and math.sqrt((blobs.cx()-x1)*(blobs.cx()-x1)+(blobs.cy()-y1)*(blobs.cy()-y1))<R_tail:
            m1 = blobs.pixels()
            blob = blobs
    #如果像素点不为0，代表一定找到车头像素，则对其画框与记录坐标值
    if(m1>0):
        img.draw_rectangle(blob.rect()) #画框
        x0 = blob.cx()
        y0 = blob.cy()
        #img.draw_circle(int(blob.cx()), int(blob.cy()),int(10))
        img.draw_cross(blob.cx(), blob.cy()) #画十字

    #******************************************信标*************************************************#
    m1 = 0
    for blobs in img.find_blobs([target], pixels_threshold=10, area_threshold=10, merge=True):
        if(m1<blobs.pixels()):
            m1 = blobs.pixels()
            blob = blobs
    if(m1>0):
        #img.draw_rectangle(blob.rect())
        x2 = blob.cx()
        y2 = blob.cy()
        w = blob.w()
        m = m1
        #img.draw_cross(blob.cx(), blob.cy())


    #******************************************障碍*************************************************#

    for blobs in img.find_blobs([target_miss], pixels_threshold=miss_min, area_threshold=miss_min, merge=True):
        missNode_m = blobs.pixels()
        if missNode_m < miss_max and blobs.cx()>120-yellow_line_not and blobs.cx()<120+yellow_line_not and blobs.cy()>120-yellow_line_not and blobs.cy()<120+yellow_line_not:
            if ( abs(x2-blobs.cx())>10 or abs(y2-blobs.cy())>10 ) and ( abs(x1-blobs.cx())>5 or abs(y1-blobs.cy())>5 ) and ( abs(x0-blobs.cx())>5 or abs(y0-blobs.cy())>5 ):#排除信标
                img.draw_rectangle(blobs.rect())
                img.draw_cross(blobs.cx(), blobs.cy())
                target_miss_list+=[(blobs.cx(),blobs.cy(),missNode_m)]


    #*****************************************计算避障坐标*******************************************#
    car_x=(x0+x1)/2
    car_y=(y0+y1)/2
    middle_lamp_long=math.sqrt((x2-car_x)*(x2-car_x)+(y2-car_y)*(y2-car_y))


    img.draw_cross(int(x2),int(y2))
    temp_x2=x2
    temp_y2=y2
    print("lamp_count",lamp_count)
 #   R=math.sqrt(m)/2
    if R==0 :
        R = 10
    miss_middle_long=math.sqrt(abs(middle_lamp_long*middle_lamp_long-R*R))
    if miss_middle_long==0:
        miss_middle_long=R
    cotA1=miss_middle_long/R
    cotA3=R/miss_middle_long
    k1 = (x1-x0)*(y2-y0)-(x2-x0)*(y1-y0)
    print("k1=",k1)
    #选择左转还是右转
    #if k1<0 and  middle_lamp_long>math.sqrt(m)*5:
        #change_x2=(temp_x2*cotA1+x1*cotA3+y1-temp_y2)/(cotA3+cotA1)
        #change_y2=(temp_y2*cotA1+y1*cotA3+temp_x2-x1)/(cotA3+cotA1)
    #else:
    change_x2=(x1*cotA3+temp_x2*cotA1+temp_y2-y1)/(cotA1+cotA3)
    change_y2=(y1*cotA3+temp_y2*cotA1+x1-temp_x2)/(cotA1+cotA3)


    img.draw_circle(int(temp_x2), int(temp_y2),int(R))
    img.draw_cross(int(x2),int(y2))
    img.draw_cross(int(temp_x2),int(temp_y2))
    img.draw_line([int(car_x),int(car_y),int(change_x2),int(change_y2)])

    print("R=",R)
    print("x2",x2)
    print("y2",y2)

    img.draw_cross(int(change_x2),int(change_y2))  #将切点十字画出来
        #x2变成了偏移后的坐标

    #************************************计算车长、车尾、车灯长度*************************************#
    car_long = math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))  #车长
    head_lamp_long = math.sqrt((change_x2-x1)*(change_x2-x1)+(change_y2-y1)*(change_y2-y1))  #车头与灯长
    tail_lamp_long = math.sqrt((change_x2-x0)*(change_x2-x0)+(change_y2-y0)*(change_y2-y0))  #车尾与灯长

    #*****************************************计算减速距离*******************************************#
    if middle_lamp_long-pre_middle_lamp_long>R*6:#证明换灯了,并计算出灭掉这个信标所需要的减速距离
        miss_long=middle_lamp_long/2
    print("miss_long ",miss_long)

    if middle_lamp_long<miss_long:#设置减速距离
        speed="-"
    else:
        speed="+"

    pre_middle_lamp_long=middle_lamp_long#记录前一次车灯距离


    #*******************************************计算角度********************************************#
    k = (x1-x0)*(change_y2-y0)-(change_x2-x0)*(y1-y0)  #获取三个坐标值后，可以通过这个公式算出应该左转还是右转，这里多次测试，不用检查错误了
    c =  car_long*tail_lamp_long
    if c!=0:
        angle_float = math.acos(((x1-x0)*(change_x2-x0)+(y1-y0)*(change_y2-y0))/c)*57.29578 #通过点积计算角度范围【0,180】，通过大小得出应该转多少角度
    if not math.isnan(angle_float) :#防止变成nan
        angle_int = int(angle_float)  #记录角度大小，将angle_float转换为int

    #********************************************数据发送*******************************************#
    if k<=0:  #k>0则左转
        if angle_int>angle_min and angle_int<angle_max:   #如果大于72小于108转大角度
            angle_char_send = 109
        elif angle_int<=angle_min:
            angle_char_send = int(angle_int/6+97)    #发送转的角度
        else :
            angle_char_send = int(110+(180-angle_int)/6)

    if k>0:  #k>0则右转
        if angle_int>angle_min and angle_int<angle_max:
            angle_char_send =77
        elif angle_int<=angle_min:
            angle_char_send = int(angle_int/6+65)
        else :
            angle_char_send = int(78+(180-angle_int)/6) #s最小

    if angle_int<=6: #如果小于5度，直行
        angle_char_send = 36  #35为#
    if angle_int>=174:
        angle_char_send = 37  #36为$

    print("k = angle_char_send = ",k)

    #if not big_flag:
        #if angle_char_send>=100 and angle_char_send <= 107:        #if big_flag and angle_char_send>='d' and angle_char_send <= 'k'
            #angle_char_send = 99
        #if angle_char_send>=68 and angle_char_send <= 75:          #if big_flag and angle_char_send>='D' and angle_char_send <= 'K'
            #angle_char_send = 67
        #if angle_char_send>=113 and angle_char_send <= 120:        #if big_flag and angle_char_send>='q' and angle_char_send <= 'x'
            #angle_char_send = 112
        #if angle_char_send>=81 and angle_char_send <= 88:          #if big_flag and angle_char_send>='Q' and angle_char_send <= 'X'
            #angle_char_send = 80

    #**************************************避障算法*************************************************#
    print ("car_long",car_long)
    cmd_miss = '!'#初始化为'!'
    for i in target_miss_list :
        x=i[0]
        y=i[1]
        m=math.sqrt(i[2])
        car_missNode_long=math.sqrt((car_x-x)*(car_x-x)+(car_y-y)*(car_y-y))#小车到障碍点的距离
        #计算角度#
        tail_missNode_long = math.sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0))  #车尾与灯长
        k = (x1-x0)*(y-y0)-(x-x0)*(y1-y0)
        c =  car_long*tail_missNode_long
        if c!=0:
            miss_angle = math.acos(((x1-x0)*(x-x0)+(y1-y0)*(y-y0))/c)*57.29578 #通过点积计算角度范围【0,180】，通过大小得出应该转多少角度
        if not math.isnan(miss_angle) :#防止变成nan
            miss_angle = int(miss_angle)  #记录角度大小，将angle_float转换为int
 #       print("r",x,y,m,m*15,car_missNode_long,miss_angle)
        if car_missNode_long<64 and miss_angle<12:#车头避障参数
            if k>0 :#边避障x
                cmd_miss='{' #"{" 左转
            if k<0 :
                cmd_miss='}' #"}" 右转
            img.draw_line([int(x),int(y),int(car_x),int(car_y)])
        #    print("miss_angle",car_missNode_long)
        #    print("!!!")
        if car_missNode_long<64 and miss_angle>164:#车尾避障参数
            if k>0 :
                cmd_miss='[' #"[" 右转
            if k<0 :
                cmd_miss=']' #"]" 左转
         #   print("miss_angle",miss_angle)
          #  print("!!!")
    target_miss_list=[]
    #if (angle_char_send >= 71 and angle_char_send <= 90) or (angle_char_send >= 103 and angle_char_send <= 122):
     #   angle_char_send = angle_char_send+1
    if (angle_char_send >= 71 and angle_char_send <= 77) or (angle_char_send >= 84 and angle_char_send <= 90) or (angle_char_send >= 103 and angle_char_send <= 109) or (angle_char_send >= 116 and angle_char_send <= 122):
        angle_char_send = angle_char_send+1


    #***************************************判断是否需要延迟减速**************************************#
    if speed == '+' and speed_temp == '-':
        speed_flag = 1
    if speed_flag_count > 7:
        speed_flag_count = 0
        speed_flag = 0
    if speed_flag == 1:
        speed_flag_count = speed_flag_count+1
        speed = '-'
    speed_temp = speed


    #*************************************发送命令**************************************************#

                #转向角度                +加减速      +避障        +数据包序号
    print("cmd "+chr(angle_char_send-1)+speed+" "+cmd_miss+" "+str(count))
 #   uart.write("a"+speed+cmd_miss+str(count))
    uart.write(chr(angle_char_send-1)+speed+cmd_miss+str(count))
    count+=1
    print("angle_int",str(angle_int))

    car_m = m


    #**************************************防止死区*************************************************#
    #print("!!!death_count!!! ",death_count)

    #if death_count>70:#进入死区
        #print("death!!!death!!!death!!!death!!!death!!!death!!!death!!!")
        #uart.write('#'+speed+cmd_miss+str(count))
        #time.sleep(500)
        #death_count=0#重置死区计数

    ##检查速度状态
    #if pre_speed=="-" and speed=="-":
        #death_count+=1
    #else:
        #death_count=0#刷新死区计数
    #pre_speed=speed


