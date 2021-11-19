#include "libHelloSLAM.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>
#include <list>
#include <mutex>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <vector>
#include <algorithm>
#include <set>
#include <string>
#include<math.h>


using namespace std;


//判断一个点是否在三角形内部 true表示在三角形内部，false表示不在三角形内部，通过判断当前点与三角形的3个顶点的内角和是否是360，确定该点是否在三角形的内部；
//如果内角和等于360，则在三角形的内部，否则不在三角形的内部
bool pointIntran(vector<pair<double,double>> &tran,pair<double,double>&point){
      vector<pair<double ,double >> line_vec;
      for(int i=0;i<3;i++){
          double tempx= tran[i].first -point.first ;
          double tempy= tran[i].second -point.second ;
          line_vec.push_back(make_pair(tempx,tempy));
      }
      double dot_oab =line_vec[0].first*line_vec[1].first+line_vec[0].second*line_vec[1].second;
      double dot_oac =line_vec[0].first*line_vec[2].first+line_vec[0].second*line_vec[2].second;
      double dot_obc =line_vec[1].first*line_vec[2].first+line_vec[1].second*line_vec[2].second;

      double temp_oa =line_vec[0].first*line_vec[0].first+line_vec[0].second*line_vec[0].second;
      double oa_norm =sqrt(temp_oa);

      double temp_ob =line_vec[1].first*line_vec[1].first+line_vec[1].second*line_vec[1].second;
      double ob_norm =sqrt(temp_ob);

      double temp_oc =line_vec[2].first*line_vec[2].first+line_vec[2].second*line_vec[2].second;
      double oc_norm =sqrt(temp_oc);

      double thta1 =acos(dot_oab/(oa_norm*ob_norm)) *180/3.14;
      cout<<"thta1 ="<<thta1<<endl;
      double thta2 =acos(dot_oac/(oa_norm*oc_norm)) *180/3.14;
      cout<<"thta2 ="<<thta2<<endl;
      double thta3 =acos(dot_obc/(ob_norm*oc_norm)) *180/3.14;
      cout<<"thta3 ="<<thta3<<endl;
      double result =thta1+thta2+thta3;
      cout<<"result ="<<result<<endl;
      if(result<361&&result>358){
           return true;
      }
      return false;

}
//判断两个三角形是否相交，true表示相交，否则不相交
bool twoTran(vector<pair<double,double>> &tran1,vector<pair<double,double>>&tran2){

    static int count2_flag=0;
        for(int ij=0;ij<3;ij++){
            cout<<"tran2[ij].first ="<<tran2[ij].first<<endl;
            cout<<"tran2[ij].first ="<<tran2[ij].second<<endl;
            pair<double,double>temp_kj2 =make_pair(tran2[ij].first,tran2[ij].second);
                bool temp_ll2 =pointIntran(tran1,temp_kj2);
                if(temp_ll2==true){
                     count2_flag++;
                }
            
        }

        static int count_flag=0;
        for(int i=0;i<3;i++){
            pair<double,double>temp_kj =make_pair(tran1[i].first,tran1[i].second);
            
                bool temp_ll =pointIntran(tran2,temp_kj);
                if(temp_ll==true){
                     count_flag++;
                }
            
        }
    if(count_flag==0&&count2_flag==0){
         return false; 
    }else if((count_flag>0&&count_flag<3)||(count2_flag>0&&count2_flag<3)){
       return true; 
    }else if(count_flag == 3||count2_flag==3){
        return false;
    }

        
}

int main(int argc, char **argv) {
    
 
  /*hash映射
   */
  

//判断一个点是否在三角形内部
vector<pair<double,double>> tran1;
vector<pair<double,double>>tan2;
pair<double,double> point1 = make_pair(0.0,1.0);//设置三角形1的顶点
pair<double,double> point2 = make_pair(1.0,0.0);
pair<double,double> point3 = make_pair(-1.0,0.0);
tran1.push_back(point1);
tran1.push_back(point2);
tran1.push_back(point3);
pair<double,double> point21 = make_pair(0.5,0.5);//设置三角形2的顶点
pair<double,double> point22 = make_pair(2.0,2.0);
pair<double,double> point23 = make_pair(3.0,0.5);
tan2.push_back(point21);
tan2.push_back(point22);
tan2.push_back(point23);

bool tren = twoTran(tran1,tan2);//求两个三角形是否相交函数调用
if(tren){
     cout<<"两个三角形相交"<<endl;
}else{
    cout<<"两个三角形不相交"<<endl;
}
  return 0;
}
