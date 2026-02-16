# ROSを使うためのもの
import rclpy
from rclpy.node import Node

#地形輪郭のデータ(class:Field)を作って表示
import numpy as np
import matplotlib.pyplot as plt
import functools

#シミュレーターを作る
import pandas as pd
import dataclasses


#@title 1.0.地形輪郭のデータ(class:Field)を作って表示
# 前のノートから引っ張ってきたのをちょっと改良した

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

class Field():
    OUTERFRAME_CONTOUR_FLAGs = np.array([[5424,1],[1800,1],[700,-1],[700,-1],[738,-1],[2500,1],[1462,1],[3424,1]])
    OUTERFRAME_ORIGIN = np.array([1712,-3462])

    CENTERNOTE_CONTOUR_FLAGs = np.array([[700,-1]]) ## ここでの距離の単位mm
    CENTERNOTE_ORIGIN = np.array([-350,-350])
    def __init__(self):
        self.OUTERFRAME_coords = cvt_flag2coords(Field.OUTERFRAME_ORIGIN,1,Field.OUTERFRAME_CONTOUR_FLAGs)
        self.CENTERNOTE_coords = cvt_flag2coords(Field.CENTERNOTE_ORIGIN,1,Field.CENTERNOTE_CONTOUR_FLAGs)

    def match_flag2coutour(self,flags):
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


#@title 2.0.シミュレーターを作る

item = ["t","robot.a.x","robot.a.y","robot.v.x","robot.v.y","robot.x","robot.y",
        "robot.angular_a","robot.angular_v","robot.angular",
        "radar.coord.x","radar.coord.y","radar.angular",
        "radar.r","spot.x","spot.y"]
report = pd.DataFrame(columns = item)


@dataclasses.dataclass
class LiDar:
    f: float = 0.025 # [s/回転]
    sampling_rate: int = 1440 # [標本数/回転]
    HFOV: float = 3/4*2*np.pi # 水平視野角

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
report.loc[0,("robot.x","robot.y","robot.angular")] = (700,700,np.pi/4)
report.loc[0,("robot.v.x","robot.v.y","robot.angular_v")] = (5000,-5000,0)

def complete_report(report):
    dt = LiDar.f/LiDar.sampling_rate
    a = report.iloc[:-1,[1,2,7]].values
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

def complete_radar_loc(report,fixed = (300,0)):
    fixed_r,fixed_theta = fixed
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
                            np.concatenate([field.CENTERNOTE_coords[:-1].reshape(-1,1,2),field.CENTERNOTE_coords[1:].reshape(-1,1,2)],axis=1)]) #(20, 2:連続分, 2:xy分)
    base_coords = report.loc[:,["radar.coord.x","radar.coord.y"]].values #(1440:時間分,2:xy分)
    theta = report["radar.angular"].values.astype(np.float64) #(1440:時間分,)
    pairs_t = pairs.reshape(1,*pairs.shape)-base_coords.reshape(base_coords.shape[0],1,1,base_coords.shape[1]) #(1440:時間分,20:辺分,2:連続分,2:xy分)
    n = np.array([np.cos(theta),np.sin(theta)]).T # (1440,2)
    n_v = np.array([-np.sin(theta),np.cos(theta)]).T # (1440,2)
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
    report[["spot.x","spot.y"]] = (base_coords.reshape(-1,1,2)+intersection_coords)[(np.arange(r_min.size),r_min_ind)]


#@title 3.0.ノイズの再現

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

# @title 一回Estimateで辺と頂点を抽出してみたい(モーションディストーションなし)
def est_err(point_coords):
    """Lidarの動きよりレーダー角速度の方が大きいことを根拠に、距離からレーダー角速度のみを用いて2次元に投影した図形の辺は直線近似ができる、ノイズは真の値に対して+ohmや-ohmを取るとする
    と考えていた時期がありました。たぶん全体的に微妙に曲線になっていてフィットが厳しい。いや移動平均で何とかなるか？
    まずrが連続してる部分でrから直接凹凸を取り出すのもできそうではある。そもそも連続した複数点の重心が尖っている点の内側になることを利用できるか
    閾値→フィールド上の最近接点が700mmなので辺のノイズ幅を350mmまでと考える"""
    distract_threshold = 10
    start = 0
    end = 0
    result = []
    while True:
        for end in range(start+2, point_coords.shape[0]):
            M = np.mean(point_coords[start:end], axis=0)
            s_xy = np.sum((point_coords[start:end, 1]-M[1])*(point_coords[start:end, 0]-M[0]))
            s_x = np.sum((point_coords[start:end,0]-M[0])**2)
            print(s_xy)
            print(s_x)
            gap = ((point_coords[end:end+distract_threshold])-M) @ np.array([[-s_xy],[s_x]])
            print(gap)
            if np.all(gap>0) or np.all(gap<0):
                result.append(end)
                start = end
                break
        if start+2 >= point_coords.shape[0]:
            break
    print(result)

def cvt_polar2xy(rs,thetas):
    return rs.reshape((-1,1)) * np.array([np.cos(thetas),
                                          np.sin(thetas)]).T

class Jikoiti(Node):

    def __init__(self):
        super().__init__('jikoiti')
        field = Field()
        complete_report(report)
        complete_radar_loc(report)
        sample_radar(report,field)
#       print(report)
        r = report.loc[:,"radar.r"].values.astype(float)
        theta = np.linspace(0,2*np.pi,LiDar.sampling_rate)
#        plt.scatter(*(lidar_scan_noise(r).reshape(-1,1)*np.array([np.cos(theta),np.sin(theta)]).T).T,s=1)
#        plt.gca().set_aspect("equal")

        #@title 前後平均ベクトル内積を見てみる
        noisy_r = lidar_scan_noise(r).reshape(-1,1)*np.array([np.cos(theta),np.sin(theta)]).T
        #plt.scatter(*(noisy_r[1:]-noisy_r[:-1]).T)
        d_noisy_r = noisy_r[1:]-noisy_r[:-1]
        div_ind = np.where(np.any(np.abs(d_noisy_r) >= 350, axis=-1))
        blocks = []
        seq = 10

        size = d_noisy_r.shape[0]

        former = np.arange(size-2*seq).reshape(-1,1) + np.arange(seq).reshape(1,-1)
        later = np.arange(seq, size-seq).reshape(-1,1) + np.arange(seq).reshape(1,-1)

        former = d_noisy_r[(former.reshape(-1),)].reshape(*former.shape, 2)
        later = d_noisy_r[(later.reshape(-1),)].reshape(*later.shape, 2)

        print(former.shape)
        norm_former = former / np.linalg.norm(former,axis=-1).reshape(-1,seq,1)
        norm_later = later / np.linalg.norm(later,axis=-1).reshape(-1,seq,1)
        print(norm_former)

        m_norm_former = norm_former.mean(axis=-2)
        m_norm_later = norm_later.mean(axis=-2)
        print(m_norm_former.shape)
        m_norm_dot = np.sum(m_norm_former * m_norm_later, axis=-1)
        print(m_norm_dot)
        dots_size= m_norm_dot.size
#        plt.scatter(np.arange(dots_size),m_norm_dot,s =1)
        #print(blocks)
#        plt.plot([0,dots_size],[0,0],alpha=0.2)
        for ind in div_ind[0]:
            y = np.array([-0.5,1])
#            plt.gca().fill_betweenx(y,[ind-seq,ind-seq],[ind+seq,ind+seq],fc="red",alpha=0.3)
#        plt.gca().set_ylim([-0.5,1.1])
#        plt.show()

        #回帰直線を引く

        r = report.loc[:,"radar.r"].values
#        plt.scatter(report.index.values,r)
#        plt.gca().set_ylim(0,None)
        theta = np.linspace(0,2*np.pi,LiDar.sampling_rate)
        est_coords = r.reshape(-1,1)*np.array([np.cos(theta),np.sin(theta)]).T
        print(est_coords)
        est_err(est_coords)
#        plt.show()

        # @title 一回Estimateで辺と頂点を抽出してみたい(モーションディストーションなし)
        plt.title("predicted_value_from_Lidar")
        thetas = np.linspace(np.pi/2-LiDar.HFOV/2,np.pi/2+LiDar.HFOV/2,1440)
        points_coord = cvt_polar2xy(r,np.linspace(np.pi/2-LiDar.HFOV/2,np.pi/2+LiDar.HFOV/2,1440))
        plt.scatter(*points_coord.T)
        plt.gca().set_xlim([-4000,4000])
        plt.gca().set_ylim([0,4000])
        plt.gca().set_aspect('equal')

        # まず視界に映る地形を分ける
        gap_inds = np.where(np.abs(r[1:]-r[:-1])>=750)[0] + 1
        print(gap_inds)
        gap_inds = [None,*gap_inds,None]
        clusters = []
        for gap_ind1,gap_ind2 in zip(gap_inds[:-1],gap_inds[1:]):
            plt.scatter(*cvt_polar2xy(r[slice(gap_ind1,gap_ind2)],thetas[slice(gap_ind1,gap_ind2)]).T)
            clusters.append(cvt_polar2xy(r[slice(gap_ind1,gap_ind2)],thetas[slice(gap_ind1,gap_ind2)]))

        # 傾きを二つに分ける
        # 一度n-1番目から見たn番目の位置ベクトルをだして, gap_ind番目の部分は入れないようにする
        delta = (points_coord[1:]-points_coord[:-1])#[bool_base]
        dir = delta[:,1]/delta[:,0]
        result =[]
        for data in [dir[dir>0],dir[dir<0]]:
            # 四分位数の計算
            Q1 = np.percentile(data, 25)  # 第1四分位数
            Q3 = np.percentile(data, 75)  # 第3四分位数
            IQR = Q3 - Q1

            # 外れ値(IQR法)の範囲
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR
            result.append(np.mean(data[(data > lower_bound) & (data<upper_bound)]))

        ##
        while True:
            for cluster in clusters:
                for ct in cluster:
                    print(ct)
            break
        print(result)
        plt.show()



def main(args=None):
    rclpy.init(args=args)

    jikoiti = Jikoiti()

if __name__ == '__main__':
    main()
