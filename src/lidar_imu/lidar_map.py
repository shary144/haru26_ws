import numpy as np
import matplotlib.pyplot as plt

#てきとうなフィールド作り
#(xg,yg,xl,yl)とする
#回転が正になるようにする。
def Reclist(xl,yl,xg,yg):
  return [[xg-0.5*xl,yg-0.5*yl,xg+0.5*xl,yg-0.5*yl],
          [xg+0.5*xl,yg-0.5*yl,xg+0.5*xl,yg+0.5*yl],
          [xg+0.5*xl,yg+0.5*yl,xg-0.5*xl,yg+0.5*yl],
          [xg-0.5*xl,yg+0.5*yl,xg-0.5*xl,yg-0.5*yl]]
    
    
# 2/2(はじめ
# tate =0.3
# yoko =0.3
# high =0.2
# wd=0.01

# object_list =[[yoko,wd, high,yoko/2.,0.,high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [yoko,wd, high,yoko/2.,tate,high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [wd,tate, high,yoko,tate/2.,high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8]]
# おわり)

# 春ロボフィールド（はじめ

tate =7.0
yoko =3.5
high =0.089
wd=0.038
wall = 2.5
box_r = 0.556
box_f = 0.7
box_high = 0.862

# #             {length_x, l_y, l_z,    x, y, z(center),    qx,qy,qz,qw,         r,g,b,a}
object_list =[[yoko, wd, high, yoko/2, wd/2, high/2, 0, 0,  0, 1,    1, 1, 1, 0.8],
              [wd, tate - 2 * wd, high, yoko - wd/2, tate/2, high/2,  0, 0, 0, 1, 1, 1, 1, 0.8],
              [yoko, wd, high, yoko/2, tate - wd/2, high/2,  0, 0,  0, 1,   1, 1, 1, 0.8],
              [wd, tate - 2 * wd, high, wd/2, tate/2, high/2,     0, 0,  0, 1,    1, 1, 1, 0.8],
              [wall, wd, high, (wall + wd)/2, 1.462 + wd + wd/2, high/2,  0, 0,  0, 1,    1, 1, 1, 0.8],
              [wall, wd, high, yoko - wd - wall/2, tate - wd - 1.462 - wd/2, high/2,  0, 0,  0, 1,    1, 0, 0, 0.8],
              [box_r, box_r, box_high, 0.924 + box_f/2, tate - 1.462 - wd * 2 - box_f/2, high/2, 0, 0,  0, 1,   0, 0, 1, 0.8],
              [box_r, box_r, box_high, yoko/2, tate/2, high/2, 0, 0,  0, 1, 1, 0, 0, 0.8],
              [box_r, box_r, box_high, wd + wall - box_f/2, wd * 2 + 1.462 + box_f/2, high/2, 0, 0, 0, 1, 1, 1, 0, 0.8]
              ]

#おわり）




# wd=0.01
# ume =1.2
# yoko=3.60
# tate=3.05
# high=0.1
# # map
# #             {length_x, l_y, l_z,    x, y, z(center),    qx,qy,qz,qw,         r,g,b,a}

# object_list =[[  yoko,wd  ,high,    yoko/2,    0.0,  high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ yoko,wd,  high,    yoko/2,  tate,  high/2.,     0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ wd,tate,    high,    yoko, tate/2,    high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [wd,tate,high,0.0,tate/2,high/2,0,0,0,1,1,0,0,0.8],
#               [ume,ume,0.2,1.27+0.6,1.05+0.6,0.1,0,0,0,1,1,0,0,0.8]]


# object_list =[[ wd,  yoko,  high,    0.0,    0.0,  high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ wd,  yoko,  high,    3.070,  0.0,  high/2.,     0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ tate, wd,   high,    1.5, -1.7,    high/2.,      0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ tate, wd,   high,    1.5,  1.7,    high/2.,       0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ kfs,  kfs,   kfs,    1.6,  0.6,   kfs/2,       0, 0,  0, 1,    1, 0, 0, 0.8],
#               [ kfs,  kfs,   kfs,    1.6, -0.6,  kfs/2,      0, 0,  0, 1,    1, 0, 0, 0.8]]
          
 
linesgroup=[]

for i in range(len(object_list)):
 
  print(object_list[i])
  
  # print("")
  linesgroup.append(Reclist(object_list[i][0],object_list[i][1],object_list[i][3],object_list[i][4]))

#さらにここで追加を入れる。斜めの線など
# linesgroup.append([[1000,1000,3000,3000],
#                    [0,6000,3000,3000]])
#結合

lines=[line for sublist in linesgroup for line in sublist]

np.save('lidar_1211',lines)
np.savetxt('lidar_1211.csv', lines, delimiter=',')

def ConvertPoints(LINES):
  POINTS=[]
  n=100
  for li in LINES:
      ld=np.sqrt((li[0]-li[2])**2+(li[1]-li[3])**2)
      pl=np.array([np.linspace(li[0],li[2],round(n*ld+1)) ,np.linspace(li[1],li[3],round(n*ld+1))]).T
      for p in pl:
          POINTS.append(p)
  POINTS=np.array(POINTS).T
  return POINTS



fig=plt.figure(figsize=(6,10))
plt.gca().set_aspect('equal', adjustable='box') # アスペクト比を1に設定


p=ConvertPoints(lines)

plt.plot(p[0],p[1],'o',markersize=2)
plt.savefig("field.jpeg")
# plt.show()
print(p.shape)


# {0.03, 3.49, 0.2, 0.0, 0.0, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}
# {0.03, 3.49, 0.2, 3.07, 0.0, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}
# {3.07, 0.03, 0.2, 1.5, -1.7, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}
# {3.07, 0.03, 0.2, 1.5, 1.7, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}
# {0.35, 0.35, 0.2, 1.6, 0.6, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}
# {0.35, 0.35, 0.2, 1.6, -0.6, 0.1, 0, 0, 0, 1, 1, 0, 0, 0.8}