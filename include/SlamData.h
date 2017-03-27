#include <vector>
#include <unordered_map>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/unordered_map.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template <class Archive>
        void save(Archive& ar, const ::cv::Mat& m, const unsigned int version) {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar& m.cols;
            ar& m.rows;
            ar& elem_size;
            ar& elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar& boost::serialization::make_array(m.ptr(), data_size);
        }

        /** Serialization support for cv::Mat */
        template <class Archive>
        void load(Archive& ar, ::cv::Mat& m, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;

            ar& cols;
            ar& rows;
            ar& elem_size;
            ar& elem_type;

            m.create(rows, cols, elem_type);

            size_t data_size = m.cols * m.rows * elem_size;
            ar& boost::serialization::make_array(m.ptr(), data_size);
        }
    }
}

namespace boost {
    namespace serialization {

        template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
        inline void save(Archive& ar, const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                         const unsigned int version) {
            int rows = g.rows();
            int cols = g.cols();

            ar& rows;
            ar& cols;
            ar& boost::serialization::make_array(g.data(), rows * cols);
        }

        template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
        inline void load(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g, const unsigned int version) {
            int rows, cols;
            ar& rows;
            ar& cols;
            g.resize(rows, cols);
            ar& boost::serialization::make_array(g.data(), rows * cols);
        }

        template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
        inline void serialize(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                              const unsigned int version) {
            split_free(ar, g, version);
        }
    }
}

BOOST_SERIALIZATION_SPLIT_FREE(::cv::KeyPoint)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::KeyPoint */
        template <class Archive>
        void save(Archive& ar, const cv::KeyPoint& p, const unsigned int version) {
            ar& p.pt.x;
            ar& p.pt.y;
            ar& p.size;
            ar& p.angle;
            ar& p.response;
            ar& p.octave;
            ar& p.class_id;
        }

        /** Serialization support for cv::KeyPoint */
        template <class Archive>
        void load(Archive& ar, cv::KeyPoint& p, const unsigned int version) {
            ar& p.pt.x;
            ar& p.pt.y;
            ar& p.size;
            ar& p.angle;
            ar& p.response;
            ar& p.octave;
            ar& p.class_id;
        }
    }
}

class KeyFrameInfo {
public:
    int id;
    Eigen::Matrix4d Tcw; //Tcw
    int imageId;
    std::vector<cv::KeyPoint> keyPoints;  // idx : kpId
    std::vector<int> mapPointIds;         // idx : kpId
    cv::Mat descriptors;                  // idx : kpId
    std::vector<float> invLevelSigma2s;   // idx : KeyPoint.octave

    int parentId;  // not found : -1
    std::set<int> childIds;
    std::vector<int> loopIds;
    std::vector<int> strongCovisibles;
    std::vector<int> covisibles;

    const Eigen::Matrix3d getRcw(){
        return Tcw.block(0, 0, 3, 3);
    }

    const Eigen::Vector3d gettwc(){
        return Tcw.block(0, 3, 3, 1);
    }

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& id;
        ar& pose;
        ar& imageId;
        ar& keyPoints;
        ar& mapPointIds;
        ar& descriptors;
        ar& invLevelSigma2s;
        ar& parentId;
        ar& childIds;
        ar& loopIds;
        ar& strongCovisibles;
        ar& covisibles;
    }
};

class MapPointInfo {
public:
    int id;
    Eigen::Vector3d pos;
    std::vector<std::pair<int, int>> kfKpIdPairs;  // KfId, KpId

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& id;
        ar& pos;
        ar& kfKpIdPairs;
    }
};

class SlamData {
public:
    std::unordered_map<int, KeyFrameInfo> keyFrameInfoMap;
    std::unordered_map<int, MapPointInfo> mapPointInfoMap;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& keyFrameInfoMap;
        ar& mapPointInfoMap;
    }
};
