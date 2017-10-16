#ifndef AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS
#define AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS

#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

namespace akira_recog_obj
{
  class recogObjMainClass : public nodelet::Nodelet
  {
  public:
    recogObjMainClass ();
    ~recogObjMainClass ();

    extern ros::Publisher pub;
    extern ros::Subscriber sub;

    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input );

  };
}

#endif

/*****
・ワンタイムインクルード(インクルードガード)
ヘッダファイルが複数回インクルードされるのを防ぐ

#ifndef, #define の後の文字列は他のヘッダファイルと重複しないものを指定する．
2度目以降インクルードされるときは，最初の #ifndef ではじかれる．

・仮想関数
仮想関数はメンバ関数の宣言に virtual キーワードをつけることで宣言できる．
仮想関数は派生クラスで再定義することで，クラスの振る舞いをカスタマイズできる．
*****/
