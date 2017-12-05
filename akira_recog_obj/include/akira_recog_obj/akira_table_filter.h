#ifndef AKIRA_RECOG_OBJ_H_TABLE_FILTER_CLASS
#define AKIRA_RECOG_OBJ_H_TABLE_FILTER_CLASS

#include <stdio.h>

namespace akira_recog_obj
{
  class value
  {
    double x;
    double y;
    double z;
    
  public:
    value () { x = 0; y = 0; z = 0; }
    ~value () {}
    
    void setAll( double num ) { x = num; y = num; z = num; } 
    void setX ( double num ) { x = num; }
    void setY ( double num ) { y = num; }
    void setZ ( double num ) { z = num; }

    double getX () { return x; }
    double getY () { return y; }
    double getZ () { return z; }
    
  };
  
  class table_filter
  {
  public:
    akira_recog_obj::value max;
    akira_recog_obj::value min;
    table_filter () {}
    ~table_filter () {}
  };
}

#endif
