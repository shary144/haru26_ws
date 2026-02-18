#とりあえず関数定義の部分だけ作ってしまいます。

# ---import---
import numpy as np #配列を用いる
import matplotlib.pyplot as plt #実用上ではあまり必要ないかも
import functools #関数を高度につかえるようにするやつ？解説求む

import pandas as pd #シミュレータ用
import dataclasses #C++のclassのような書き方をできるようにする


# ---関数、classなど---
#機体の最初の座標と向き、また進む距離と曲がる方向を入手して、x, y座標に変換
#flags...進む距離と曲がる方向の書かれたn行2列のnp.array
def cvt_flag2coords(origin_coord,origin_cam:int,flags): 
    coords = [origin_coord]
    cam = origin_cam
    while True:
        for flag in flags:
            length, dcam = flag
            coords.append(coords[-1] + length * np.array([np.cos(cam*np.pi/2),np.sin(cam*np.pi/2)]))
            cam += dcam
        if cam%4 == 1:
            break
    return np.array(coords)

# [頂点フラグ] 右: -1, 左: 1

class Field(): #フィールドを作成
    OUTERFRAME_CONTOUR_FLAGs = np.array([[5424,1],[1800,1],[700,-1],[700,-1],[738,-1],[2500,1],[1462,1],[3424,1]])
    OUTERFRAME_ORIGIN = np.array([1712,-3462])

    CENTERNOTE_CONTOUR_FLAGs = np.array([[700,-1]]) ## ここでの距離の単位mm
    CENTERNOTE_ORIGIN = np.array([-350,-350])
    def __init__(self):
        self.OUTERFRAME_coords = cvt_flag2coords(Field.OUTERFRAME_ORIGIN,1,Field.OUTERFRAME_CONTOUR_FLAGs)
        self.CENTERNOTE_coords = cvt_flag2coords(Field.CENTERNOTE_ORIGIN,1,Field.CENTERNOTE_CONTOUR_FLAGs)

    def match_flag2coutour(self,flags): #フィールドとmatchするところを探す
        result = []
        n = len(flags)
        for contour_flags in [Field.OUTERFRAME_CONTOUR_FLAGs,Field.CENTERNOTE_CONTOUR_FLAGs]:
            contour_result=[]
            m = len(contour_flags)
            start_ind_candidate = np.where((contour_flags[:,0] >= flags[0,0]) & (contour_flags[:,1]==flags[0,1]))[0]
            #print(start_ind_candidate)
            for start_ind in start_ind_candidate:
                if flags[-1,0] <= contour_flags[(start_ind+n-1)%m,0]:
                    #print(contour_flags[((start_ind+np.arange(1,n-1))%m,)])
                    if np.all(flags[1:-1] == contour_flags[((start_ind+np.arange(1,n-1))%m,)]):
                        contour_result.append(start_ind)

            result.append(contour_result)
        return result

    def estimate_loc_from_rs(self,rs):
        pass

    def show(self):
        fig,ax = plt.subplots()
        ax.plot(*self.OUTERFRAME_coords.T,c = 'blue')
        ax.plot(*self.CENTERNOTE_coords.T,c = 'blue')
        ax.set_aspect('equal')
        self.ax = ax
        return ax
    
@dataclasses.dataclass
class LiDar: #LiDARの基本情報を入力
    f: float = 0.025 # [s/回転]
    sampling_rate: int = 1440 # [標本数/回転]
    HFOV: float = 3/4*2*np.pi # 水平視野角

def complete_report(report): #機体の速度と位置を求める
    dt = LiDar.f/LiDar.sampling_rate #レーダーの発射間隔
    a = report.iloc[:-1,[1,2,7]].values #[x成分, y成分, 角度成分]（以下同様）
    v0 = report.iloc[0,[3,4,8]].values 
    x0 = report.iloc[0,[5,6,9]].values
    v = [v0]
    x = [x0]
    for a_ in a:
        v.append(v[-1] + a_ * dt)
        x.append(x[-1] + v[-1] * dt)
    #print(v)
    report.loc[1:,("robot.v.x","robot.v.y","robot.angular_v")] = np.array(v[1:])
    report.loc[1:,("robot.x","robot.y","robot.angular")] = np.array(x[1:])
    #plt.scatter(*report.iloc[:,[5,6]].values.T)

def complete_radar_loc(report,fixed = (300,0)): #機体の位置をもとにLiDARの位置を求める
    fixed_r,fixed_theta = fixed #fixed...機体の中心から見たLiDARの位置(極座標)
    robot_angular = report["robot.angular"].values
    angular = np.atleast_1d(robot_angular).astype(float) + fixed_theta # Ensure angular is always a float ndarray
    #print(np.cos(angular))
    #print(fixed_r*np.array([np.cos(angular),np.sin(angular)]).T)
    report[["radar.coord.x","radar.coord.y"]] = report[["robot.x","robot.y"]].values + fixed_r*np.array([np.cos(angular),np.sin(angular)]).T
    #plt.scatter(*report[["radar.coord.x","radar.coord.y"]].values.T)
    report["radar.angular"] = report["robot.angular"] + np.linspace(0,2*np.pi, LiDar.sampling_rate)

def sample_radar(report,field:Field,fixed = (300,0)):
    fixed_r,fixed_theta = fixed
    #print(field.OUTERFRAME_coords)
    pairs = np.concatenate([np.concatenate([field.OUTERFRAME_coords[:-1].reshape(-1,1,2),field.OUTERFRAME_coords[1:].reshape(-1,1,2)],axis=1),
                            np.concatenate([field.CENTERNOTE_coords[:-1].reshape(-1,1,2),field.CENTERNOTE_coords[1:].reshape(-1,1,2)],axis=1)]) #(20:辺分, 2:連続分, 2:xy分)
    base_coords = report.loc[:,["radar.coord.x","radar.coord.y"]].values #(1440:時間分,2:xy分)
    theta = report["radar.angular"].values.astype(np.float64) #(1440:時間分,)
    pairs_t = pairs.reshape(1,*pairs.shape)-base_coords.reshape(base_coords.shape[0],1,1,base_coords.shape[1]) #(1440:時間分,20:辺分,2:連続分,2:xy分)
    n = np.array([np.cos(theta),np.sin(theta)]).T # (1440,2) # radarが飛んでいく方向をxyで表す
    n_v = np.array([-np.sin(theta),np.cos(theta)]).T # (1440,2) #radarが飛んでいく方向と垂直になるベクトル(90度回転)
    s = -np.sum(pairs_t[:,:,0]*n_v.reshape(-1,1,2),axis=-1)/np.sum((pairs_t[:,:,1]-pairs_t[:,:,0])*n_v.reshape(-1,1,2),axis=-1) ##(1440:時間分,20:辺分)
    ss = np.concatenate([(1-s).reshape(*s.shape,1),s.reshape(*s.shape,1)],axis = -1) #(1440,20,2:連続分)
    ## 時間ごとの20本の各辺(直線)と角度theta(t)の傾きの直線との交点のlidarの位置基準の相対位置
    intersection_coords = np.sum(pairs_t*ss.reshape(*ss.shape,1),axis=-2) #(1440,20,2:xy分) ##ここはあってる
    ## 検算(ここが0かそれに近い値になればよい、ならなければ変数"s"に問題あり)
    # print(intersection_coords*np.array([-np.sin(theta),np.cos(theta)]).T.reshape(-1,1,2))
    print(intersection_coords)
    #ここで表示
    r = np.sum(intersection_coords * np.array([np.cos(theta),np.sin(theta)]).T.reshape(-1,1,2),axis=-1) # (1440,20)
    r[~((r>=0)&(0<=s)&(s<=1))] = np.inf
    if np.where(r.min(axis=1)==np.inf)[0].size != 0:
        print("注意: 欠損値があります.")
    #print(r.min(axis=1))
    #print(r.min(axis=1).shape)
    r_min = r.min(axis=1) # (1440,)
    r_min_ind = np.argmin(r,axis = 1)
    #################
    report["radar.r"] = r_min
    report[["spot.x","spot.y"]] = (base_coords.reshape(-1,1,2)+intersection_coords)[(np.arange(r_min.size),r_min_ind)] # ライダーの点が当たったとされるもの

#@title 3.0.ノイズを入れたうえでのLidarの予測値
def lidar_scan_noise(r_true_array):
    sigma0 = 0.005
    k = 0.002
    outlier_prob = 0.05
    outlier_scale = 0.1

    sigma = sigma0 + k * r_true_array
    normal_noise = np.random.normal(0, sigma)

    mask = np.random.rand(len(r_true_array)) < outlier_prob
    outlier = np.random.exponential(outlier_scale, len(r_true_array)) * mask

    return r_true_array + normal_noise + outlier

# ---実行用のコード---
item = ["t","robot.a.x","robot.a.y","robot.v.x","robot.v.y","robot.x","robot.y",
        "robot.angular_a","robot.angular_v","robot.angular",
        "radar.coord.x","radar.coord.y","radar.angular",
        "radar.r","spot.x","spot.y"]
report = pd.DataFrame(columns = item)

t = np.linspace(0, LiDar.f, LiDar.sampling_rate)
report.loc[:,"t"]= t

###初期値の設定
t_cond = report.loc[:,"t"] < 0.05
# データコンテナとしてreport(pd.DataFrame)使う
report.loc[t_cond,"robot.a.x"] = 0
report.loc[~t_cond,"robot.a.x"] = 2000
report.loc[t_cond,"robot.a.y"] = -2000
report.loc[~t_cond,"robot.a.y"] = 0
report.loc[:,"robot.angular_a"] = 100
report.loc[0,("robot.x","robot.y","robot.angular")] = (700,0,np.pi/4)
report.loc[0,("robot.v.x","robot.v.y","robot.angular_v")] = (5000,-5000,0)

# radar.rとradar.angularを送ってあげればOK
