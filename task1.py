import numpy as np
import open3d as o3d
import random

print(o3d.__version__)
print(np.__version__)


pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

# convert the point cloud to numpy array
pcd_array=np.asarray(pcd.points)
print(pcd_array.shape)

# generate random samples from numpy array
random_samples=np.random.choice(pcd_array.shape[0],4)

# find paramater of the plane (a,b,c,d)
points=pcd_array[random_samples,:]
ones=np.ones((4,1))
A=np.hstack((points,ones)) # f by 4
B=np.array([0,0,0,0])
ans = np.linalg.lstsq(A, B)
print('result from least square is')
print(type(ans))
print(len(ans))
print(ans)

# consider using the normal to points instead to form the equation
def find_plane_equation(point_cloud,random):
    '''
    find equation of plane through three points
    '''
    p1=pcd_array[random_samples[0],:].reshape(3,1)
    x1=p1[0][0]
    y1=p1[1][0]
    z1=p1[2][0]
    p2=pcd_array[random_samples[1],:].reshape(3,1)
    x2=p2[0][0]
    y2=p2[1][0]
    z2=p2[2][0]
    p3=pcd_array[random_samples[2],:].reshape(3,1)
    x3=p3[0][0]
    y3=p3[1][0]
    z3=p3[2][0]

    # find the parameters
    a1 = x2 - x1
    b1 = y2 - y1
    c1 = z2 - z1
    a2 = x3 - x1
    b2 = y3 - y1
    c2 = z3 - z1
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * x1 - b * y1 - c * z1)
    return a,b,c,d
  
def RANSAC():
    '''
    Customized Implementation of RANSAC.
    Returns an inliers dictionary consisiting of keys: number of iterations
    and values equal to the actual counts.
    Returns an inliers_dict, consists of keys: number of iterations and 
    values equal to the indices of the inliers. 
    '''
    pcd_point_cloud = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)
    # convert the point cloud to numpy array
    pcd_array=np.asarray(pcd.points)
    print(pcd_array.shape)
    
    
    
    inliers={}
    indices_dict={}
    iterations=50
    for n in range(iterations):
        random_samples=np.random.choice(pcd_array.shape[0],3)
        inliers_count=3
        inliers_indices=[]
        a,b,c,d=find_plane_equation(pcd_array,random_samples)
        for index in range(pcd_array.shape[0]):
            if index in random_samples:
                continue
            point=pcd_array[index,:].reshape(3,1)
            x=point[0][0]
            y=point[1][0]
            z=point[2][0]
            fit=x*a+y*b+z*c+d
            if fit==0:
                inliers_count+=1
                inliers_indices.append(index)
        inliers[str(n)]=inliers_count
        indices_dict[str(n)]=inliers_indices
        
    return inliers,indices_dict['49']

inliers,indices_dic=RANSAC()
print(inliers)
print(indices_dic)
print(len(indices_dic))
saved_indices=np.asarray(indices_dic)
file=np.save('saved_indices',saved_indices)
inliers=np.load(r'C:\Users\15512\Desktop\Fall 22\Fall 22 NYU\Perception\Assignments\Assignment2\Questions\Question 1')
pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])

