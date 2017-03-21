#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
  namespace serialization {

    /** Serialization support for cv::Mat */
    template<class Archive>
    void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
    {
      size_t elem_size = m.elemSize();
      size_t elem_type = m.type();

      ar & m.cols;
      ar & m.rows;
      ar & elem_size;
      ar & elem_type;

      const size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }

    /** Serialization support for cv::Mat */
    template<class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
      int cols, rows;
      size_t elem_size, elem_type;

      ar & cols;
      ar & rows;
      ar & elem_size;
      ar & elem_type;

      m.create(rows, cols, elem_type);

      size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }

  }
}

BOOST_SERIALIZATION_SPLIT_FREE(::cv::KeyPoint)
namespace boost {
  namespace serialization {

     /** Serialization support for cv::KeyPoint */
     template<class Archive>
     void save(Archive &ar, const cv::KeyPoint &p, const unsigned int version)
     {
      ar & p.pt.x;
      ar & p.pt.y;
      ar & p.size;
      ar & p.angle;
      ar & p.response;
      ar & p.octave;
      ar & p.class_id;
     }

     /** Serialization support for cv::KeyPoint */
     template<class Archive>
     void load(Archive &ar, cv::KeyPoint &p, const unsigned int version)
     {
      ar & p.pt.x;
      ar & p.pt.y;
      ar & p.size;
      ar & p.angle;
      ar & p.response;
      ar & p.octave;
      ar & p.class_id;
    }
  }
}

class KeyFrameData{
public:
  int mId;
  cv::Mat mPose;
  double mTimeStamp;
  std::vector<cv::KeyPoint> mvKeyPoint;
  std::vector<long unsigned int> mvMapPointId;

  friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive& ar, const unsigned int version)
       {
         ar & mId;
         ar & mPose;
         ar & mTimeStamp;
         ar & mvKeyPoint;
         ar & mvMapPointId;
       }

};

class MapPointData{
public:
  int mId;
  cv::Mat mPos;

  friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive& ar, const unsigned int version)
       {
         ar & mId;
         ar & mPos;
       }

};

class Data{
public:
  std::vector<KeyFrameData> mvKeyFrame;
  std::vector<MapPointData> mvMapPoint;

  friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive& ar, const unsigned int version)
       {
         ar & mvKeyFrame;
         ar & mvMapPoint;
       }
};
