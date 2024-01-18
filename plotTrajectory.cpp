//此代码更改自slam十四讲ch3的同名代码
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// path to trajectory file
string trajectory_file = "../result.txt";
//string trajectory_file = "/home/hl/project/plot_track-colmap/moon/output.txt";
//string trajectory_file = "/home/hl/project/imu_get_scale/data/track_scale.txt";
//images.txt比images (copy).txt多了第一个点，是我人为写上的，这个点的平移矩阵为0旋转矩阵为单位矩阵，即和原点位姿相同
//根据”https://blog.csdn.net/weixin_44120025/article/details/124604229“评论区知，colmap稀疏重建后的世界坐标系原点是居中过了的，原点在场景中心

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main() {

  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);
  if (!fin) {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

    char line[50000]={0};

//从文件中提取“行”
    while(fin.getline(line,sizeof(line)))
    {
        int num;
        double time, tx, ty, tz, qx, qy, qz, qw;
        std::vector<double> rest;
        //从“行”中提取“单词”
        std::stringstream word(line);
        fin >> time >> qw >> qx >> qy >> qz >> tx >> ty >> tz>>num;
        //if (time - int(time)<0.01)
        //if(num==1)
        //    std::cout<<time<<" "<<qw<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<tx<<" "<<ty<<" "<<tz<<std::endl;

/*  这里是个坑：https://blog.csdn.net/weixin_44120025/article/details/124604229
    colmap输出的images文件中的四元数Q和平移向量T，是其定义的相机坐标系下的R和t。但是，如果我们要将这些相机放在一起进行可视化的话，那么我们需要首先将其变换到世界坐标系下（这样才能统一），即：
    R’ = RT；t’ = -RTt
    实际上，colmap自身在进行可视化的时候，已经隐含了这样的一个变换过程了，但是如果你去点击可视化的相机，查看到的数值其实还是相机坐标系下的Q和t，极具迷惑性。    */

//        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
//        Twr.pretranslate(Vector3d(tx, ty, tz));
//        poses.push_back(Twr);

        Isometry3d Twr=Isometry3d::Identity();
        Quaterniond q=Quaterniond(qw, qx, qy, qz);
        Eigen::Matrix3d R=q.toRotationMatrix();
        //Eigen::Matrix3d Rr=R.inverse();
        Eigen::Matrix3d Rr=R.transpose();
        //Quaterniond qr=Quaterniond(Rr);
        //Isometry3d Twr(qr);
        Twr.rotate(Rr);
        Vector3d t(tx,ty,tz);
        Vector3d tr=-Rr*t;
        Twr.pretranslate(tr);
        poses.push_back(Twr);
    }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (int i = 0; i < poses.size()-1; i++) {  //一是因为n个点只能连n-1个线，否则他会自动多连一条最后一个点到原点的线 多余的那条是连接最后一个点和默认的点的（应该是000）
    //for (size_t i = 0; i < 2; i++) {
      glColor3f(0.0, 0.0, 0.0);//修改颜色
      auto p1 = poses[i], p2 = poses[i + 1];
      Vector3d Start = p1.translation();
      Vector3d End = p2.translation();
      glBegin(GL_LINES);//2点为1条线，奇数点无效
      glVertex3d(Start[0], Start[1], Start[2]);//设置定点坐标
      glVertex3d(End[0], End[1], End[2]);
      glEnd();
//      cout<<"起点"<<Start[0]<<", "<<Start[1]<<", "<<Start[2]<<endl;
//      cout<<"终点"<<End[0]<<", "<<End[1]<<", "<<End[2]<<endl;
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}
