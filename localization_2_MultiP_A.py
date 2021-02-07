import numpy as np
import concurrent.futures
from scipy.stats import hypergeom
from scipy.special import hyp1f1
from scipy.io import loadmat
from scipy.optimize import fmin,fmin_powell,fmin_cg,minimize
import inspect
from scipy import random
from  Ambi_Resolve_A import Ambi_resolve


def KLD1(x,mu_r,sigma_r):
    return 1/2*(np.linalg.norm(np.array([x[0]-mu_r[0],x[1]-mu_r[1]]))**2+2*x[2]**2)/sigma_r-np.log(x[2]**2/sigma_r)-1;

def KLD2(x,mu_a_x,mu_a_y,d_r_a,sigma_d_r_a):    
    #sigma_d_r_a = .01
    d = 0
    for i in np.arange(start=0,stop=mu_a_x.__len__(),step=1):
        d = d+ (-2*d_r_a[i]*np.sqrt(x[2]**2*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2/(2*x[2]**2))+
        np.linalg.norm(np.array([x[0]-mu_a_x[i],x[1]-mu_a_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_a[i])
    return d

def KLD3(x,mu_m_x,mu_m_y,sigma_m,d_r_m,sigma_d_r_m):    
    
    d = 0
    for i in np.arange(start=0,stop=mu_m_x.__len__(),step=1):
        d = d+(-2*d_r_m[i]*np.sqrt((x[2]**2+sigma_m[i])*np.pi/2)*
        hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2/(2*(x[2]**2+sigma_m[i])))+
        np.linalg.norm(np.array([x[0]-mu_m_x[i],x[1]-mu_m_y[i]]))**2+2*x[2]**2)/(2*sigma_d_r_m[i])
    return d

def KLD3_1(x,mu_m_x,mu_m_y,sigma_m,d_r_m):    
    sigma_d_r_m = .01
    return (-2*d_r_m*np.sqrt((x[2]**2+sigma_m)*np.pi/2)*
    hyp1f1(-1/2,1,-np.linalg.norm(np.array([x[0]-mu_m_x,x[1]-mu_m_y]))**2/(2*(x[2]**2+sigma_m)))+
    np.linalg.norm(np.array([x[0]-mu_m_x,x[1]-mu_m_y]))**2+2*x[2]**2)/(2*sigma_d_r_m)


def new_func2(func1, func2):
    return lambda x: func1(x) + func2(x)

def new_func3(func_List):
    return lambda x: sum(  f(x) for f in func_List  )

def sub_fuc_loc(Quad,i,dis,Sigma,iter_all):
    #KLD = []
            
    if Quad[i].A is False and Quad[i].convergen == False:
        # a = Quad[i].mu
        # b = Quad[i].Sigma
        KLD_local= lambda x: KLD1(x,Quad[i].mu,b)


        XX, YY, mu_m_x, mu_m_y, distance_a, distance_m, sigma_m, sigma_d_r_a, sigma_d_r_m = ([] for i in range(9))
        # YY = []
        # mu_m_x = []
        # mu_m_y = []
        # distance_a = []
        # distance_m = []
        # sigma_m =[]
        # sigma_d_r_a=[]
        # sigma_d_r_m=[]
                
        for j in np.arange(start=0,stop=Quad[i].neighbor.__len__(),step=1):
                    
            neighbor_label = Quad[i].neighbor[j]
            if neighbor_label in Quad[i].List:
                XX.append(Quad[neighbor_label].X)
                YY.append(Quad[neighbor_label].Y)
                distance_a.append(dis[i,neighbor_label])
                sigma_d_r_a.append(Sigma[i,j])

            else:         
                
                mu_m_x.append(Quad[neighbor_label].mu[0])
                mu_m_y.append(Quad[neighbor_label].mu[1])
                sigma_m.append(Quad[neighbor_label].Sigma)
                distance_m.append(dis[i,neighbor_label])
                sigma_d_r_m.append(Sigma[i,j])
                  
        KLD = lambda x: KLD3(x,mu_m_x,mu_m_y,sigma_m,distance_m,sigma_d_r_m)+KLD2(x,XX,YY,distance_a,sigma_d_r_a) +KLD_local(x)
        a = Quad[i].mu[0]
        b = Quad[i].mu[1]
        c = Quad[i].Sigma**(1/2)
        X = minimize(KLD,x0 = [a,b,c],method='nelder-mead', options={'xatol': 0.001, 'fatol': 0.001})
        Quad[i].mu[0] = X.x[0]
        Quad[i].mu[1] = X.x[1]
        Quad[i].mu_x[iter_all] = X.x[0]
        Quad[i].mu_y[iter_all] = X.x[1]
        if iter_all>10:
            if np.abs(Quad[i].mu_x[iter_all]-Quad[i].mu_x[iter_all-1])<0.0001 and np.abs(Quad[i].mu_y[iter_all]-Quad[i].mu_y[iter_all-1])<0.0001:
                Quad[i].convergen = True
                #Conve = Conve +1;
            # if Conve == n:
            #     break;
        Quad[i].Sigma = np.abs(X.x[2])
    return Quad

def localization(Quad,dis,Sigma,Leader):
    np.random.seed(1)
    n = Quad.__len__() 
    print("Localizing")
    for i in np.arange(start = 0, stop = n, step = 1):
        loc[i,:] = np.array([Quad[i].X,Quad[i].Y])
        Quad[i].mu = np.array([Quad[i].X + np.random.randint(-10, 10), Quad[i].Y + np.random.randint(-10, 10)])
        Quad[i].mu_x = np.zeros(250)
        Quad[i].mu_y = np.zeros(250)
        Quad[i].Sigma = 1
        Quad[i].convergen = False
   
    
 
    #dis = dis1
    for iter_all in np.arange(start=0,stop=250,step=1):


        def function(i): return sub_fuc_loc(Quad,i,dis,Sigma,iter_all)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=n)
        with executor:
            Q = {executor.submit(function, i) for i in np.arange(start=1, stop=n, step=1)}
        i=1
        Conve = 1
        for fut in concurrent.futures.as_completed(Q):
            Quad[i] = fut.result()[i]
            if Quad[i].convergen == True:
               Conve = Conve+1
            i = i+1
        if Conve == n:
           break;
            
        # Conve = 1
        # for i in np.arange(start=1, stop=n, step=1):           
        #     temp = sub_fuc_loc(Quad,i,dis,Sigma,iter_all)
        #     Quad[i] = temp[i]

        # for i in np.arange(start=1, stop=n, step=1):
        #     if Quad[i].convergen == True:
        #         Conve = Conve + 1
        # if Conve == n:
        #     break
            

 
        print(iter_all,250)        
    print("Finished")

    for i in np.arange(start=0,stop=n,step=1):
        Quad[i].convergen = False
    
    p = np.zeros((n, 2))
    sigma = np.zeros((n, 1))    
    for i in np.arange(start=0, stop=n, step=1):
        if i == Leader:
            p[i, :] = np.array([Quad[i].X, Quad[i].Y])
            sigma[i] = Quad[i].Sigma
        else:
            p[i, :] = np.array([Quad[i].mu[0], Quad[i].mu[1]])
            sigma[i] = Quad[i].Sigma
    return p,sigma

def init_pos(n,height):
    #height = 80
    np.random.seed(1)
    randint = np.random.randint(10, 40, size = (n*2, 2))
    pos_init = np.zeros((n,3))
    position_desire  = np.zeros((n,3))

    for i in np.arange(start=0,stop=n,step=1):
        randint1 = np.random.randint(-5, 5, size = (1, 2))
        pos_init[i,:] = np.append(randint[i+n,:],height)
        if i == 0:
            pos_init[i,:] = np.array([319000,5811500,height])
            #pos_init[i,:] = np.array([320100,5811000,height])
        else:
            pos_init[i,:] = np.array([319000+randint1[0,0],5811500+randint1[0,1],height])
            #pos_init[i,:] = np.array([320100+randint1[0,0],5811000+randint1[0,1],height])
#        pos_init[i,:] = np.array([321000+i,5814000+i,height+10])
        position_desire[i,:] = np.append(randint[i+n,:],height)

    position_desire = np.multiply(np.array([[-5, -5, height],
        [-10, -10, height],
        [5, 5, height],
        [10, 10, height],
        [-5, 5, height],
        [-10, 10, height],
        [5, -5, height],
        [10, -10, height]]),np.array([.2,.2,1]))
    return pos_init,position_desire


def get_distance(Quad,offset,Leader):
    Sigma = np.zeros((n,n))
    data = loadmat('error_data_jelena_NLOS_LOS.mat')
    data1 = data['error_LOS_truncated']
    Var = 1
    dis = np.zeros((n,n))
    Sigma = np.zeros((n,n))

    for i in np.arange(start=0,stop=n,step=1):
        if i == Leader:
            X = np.array([Quad[i].X,Quad[i].Y])+offset
        else:
            X = np.array([Quad[i].X,Quad[i].Y])

        for jj in np.arange(start=i,stop=n,step=1):
            
            Y = np.array([Quad[jj].X,Quad[jj].Y])
            value_i = random.randint(0, 20000)
            dis[i,jj] = np.linalg.norm(X-Y) 
            Sigma[i,jj] = Var
            Sigma[jj,i] = Var
            dis[i,jj] =dis[i,jj]+ random.uniform(0,1)
            # dis[i,jj] =dis[i,jj]+ (data1[value_i]-0.16)
            dis[jj,i] = dis[i,jj]
    return dis,Sigma
class Quad_class:
    pass

if __name__ == "__main__":
    n = 5
    Quad = [0] * n
    Leader = 0
    loc = np.zeros((n,2))
    pos_init,position_desire = init_pos(n,10)
    for i in np.arange(start = 0, stop = n, step = 1):
        Quad[i] = Quad_class()
        Quad[i].X = pos_init[i,0]
        Quad[i].Y = pos_init[i,1]
        loc[i,:] = np.array([Quad[i].X,Quad[i].Y])
        Quad[i].mu = np.array([Quad[0].X, Quad[0].Y])
        Quad[i].Sigma = 1
        Quad[i].mu_x = np.zeros(250)
        Quad[i].mu_y = np.zeros(250)

        if i == Leader:
            Quad[i].A = True
        else:
            Quad[i].A = False
            Quad[i].neighbor = np.array(np.arange(start=0,stop=n,step=1))
            Quad[i].neighbor = np.delete(Quad[i].neighbor,i)
            Quad[i].List = np.array([0,1,2]) # anchor list
            Quad[i].convergen = False

    offset = [0,0]
    dis,Sigma = get_distance(Quad,offset,Leader)    # noise sigma   
    print(Sigma)
    # p1,sigma1 = localization(Quad,dis,Sigma,Leader) # sigma of estimation

    # offset = [0,1]
    # dis,Sigma = get_distance(Quad,offset,Leader) 
    # p2,sigma2 = localization(Quad,dis,Sigma,Leader)

    # offset = [1,0]
    # dis,Sigma = get_distance(Quad,offset,Leader) 
    # p3,sigma3 = localization(Quad,dis,Sigma,Leader)
    # offset = [0,0]
    # p1,sigma1 = localization.localization(Quad,0,offset)
    # offset = [0,1]
    # p2,sigma2 = localization.localization(Quad,0,offset)
    # offset = [1,0]
    # p3,sigma3 = localization.localization(Quad,0,offset)
    # offset = np.array([[ 0, 1],[ 1, 0]])
    # temp = Ambi_resolve(p1,p2,p3,offset)+np.array([Quad[0].X,Quad[0].Y])
    # offset = np.array([[ 0, 1],[ 1, 0]])
    # temp = Ambi_resolve.Ambi_resolve(p1,p2,p3,offset)+np.array([Quad[0].X,Quad[0].Y])

    
    error1 = np.array([[p1[0,0]-loc[0,0],p1[0,1]-loc[0,1]],[p1[1,0]-loc[1,0],p1[1,1]-loc[1,1]],[p1[2,0]-loc[2,0],p1[2,1]-loc[2,1]],[p1[3,0]-loc[3,0],p1[3,1]-loc[3,1]],[p1[4,0]-loc[4,0],p1[4,1]-loc[4,1]]])
    # error2 = np.array([[p2[0,0]-loc[0,0],p2[0,1]-loc[0,1]],[p2[1,0]-loc[1,0],p2[1,1]-loc[1,1]],[p2[2,0]-loc[2,0],p2[2,1]-loc[2,1]],[p2[3,0]-loc[3,0],p2[3,1]-loc[3,1]],[p2[4,0]-loc[4,0],p1[4,1]-loc[4,1]]])
    # error3 = np.array([[p3[0,0]-loc[0,0],p3[0,1]-loc[0,1]],[p3[1,0]-loc[1,0],p3[1,1]-loc[1,1]],[p3[2,0]-loc[2,0],p3[2,1]-loc[2,1]],[p3[3,0]-loc[3,0],p3[3,1]-loc[3,1]],[p3[4,0]-loc[4,0],p1[4,1]-loc[4,1]]])
    print('error 1 .....')
    print(error1)
    # print('error 2 .....')
    # print(error2)
    # print('error 3 .....')
    # print(error3)        
    # print('3 anchors.....')
    # print(temp-loc)

