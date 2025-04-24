#ifndef SLAM_LYJ_POLARGRID_H
#define SLAM_LYJ_POLARGRID_H

#include "base/Base.h"

#ifndef PI
#define PI 3.14159265358979323846264338327950288419
#endif
#ifndef TWICEPI
#define TWICEPI 2 * PI
#endif

NSP_SLAM_LYJ_BEGIN
template <typename T>
class PolarGrid
{
protected:
    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

protected:
    /* data */
    Scalar minR_ = 0;
    Scalar maxR_ = 0;
    Scalar minTheta_ = 0;
    Scalar maxTheta_ = PI;
    Scalar minFi_ = 0;
    Scalar maxFi_ = TWICEPI;
    uint32_t totalSize_ = 0;
    std::vector<T> datas_;

public:
    PolarGrid(/* args */) {};
    ~PolarGrid() {};

    inline const Scalar &getMinR() const { return minR_; };
    inline const Scalar &getMaxR() const { return minTheta_; };
    inline const Scalar &getMinTheta() const { return maxR_; };
    inline const Scalar &getMaxTheta() const { return maxTheta_; };
    inline const Scalar &getMinFi() const { return minFi_; };
    inline const Scalar &getMaxFi() const { return maxFi_; };
    inline const uint32_t &getTotalSize() const { return totalSize_; };
    inline const std::vector<T> &getDatas() const { return datas_; };

    virtual const Scalar &getStepR(uint32_t _i=0) const = 0;
    virtual const Scalar &getStepThete(uint32_t _i=0) const = 0;
    virtual const Scalar &getStepFi(uint32_t _i=0) const = 0;
    virtual const uint32_t &getIndRSize() const = 0;
    virtual const uint32_t &getIndThetaSize(uint32_t _i=0) const = 0;
    virtual const uint32_t &getIndFiSize(uint32_t _i=0) const = 0;
    virtual bool getIdInGridByCoord(const Vector3 &_coord, uint32_t &_id) const = 0;
    virtual bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_id) const = 0;
    virtual bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t &_id) const = 0;
    virtual bool getIndInGridById(const uint32_t _id, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getIndInGridByCoord(const Vector3 &_coord, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getPolarByCoord(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getPolarByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getPolarById(const uint32_t _id, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getCoordByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord) const = 0;
    virtual bool getCoordByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3 &_coord) const = 0;
    virtual bool getCoordById(const uint32_t _id, Vector3 &_coord) const = 0;
    virtual bool isCoordInGrid(const Vector3 &_coord) const = 0;
    bool isPolarInGrid(const Scalar _r, const Scalar _theta, const Scalar _fi) const {
        if (_r > maxR_ || _r < minR_
            || _theta > maxTheta_ || _theta < minTheta_
            || _fi > maxFi_ || _fi < minFi_
            )
            return false;
        return true;
    }
    virtual bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const = 0;
    bool isIdInGrid(const uint32_t _id) const {
        if (_id >= totalSize_)
            return false;
        return true;
    }

    static void convertCoord2Ploar(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi) {
        _r = _coord.norm();
        if (_r < 1e-5) {
            _r = 0.0f;
            _theta = 0.0f;
            _fi = 0.0f;
            return;
        }
        _theta = acosf(_coord(2) / _f);
        _fi = atan2f(_coord(1), _coord(0));
    };
    static void convertPolar2Coord(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord) {
        _coord(0) = _r * sin(_theta) * cos(_fi);
        _coord(1) = _r * sin(_theta) * sin(_fi);
        _coord(2) = _r * cos(_theta);
    };
};

template <typename T>
class SoloPolarGrid : public PolarGrid<T>
{
protected:
    /* data */
    Scalar stepR_ = 0;
    Scalar stepTheta_ = 0.1;
    Scalar stepFi_ = 0.1;
    uint32_t indRSize_ = 0;
    uint32_t indThetaSize_ = 0;
    uint32_t indFiSize_ = 0;

public:
    SoloPolarGrid(/* args */) {};
    ~SoloPolarGrid() {};

    // 通过 PolarGrid 继承
    const Scalar& getStepR(uint32_t _i = 0) const {
        return stepR_;
    }
    const Scalar& getStepTheta(uint32_t _i = 0) const {
        return stepTheta_;
    }
    const Scalar& getStepFi(uint32_t _i = 0) const {
        return stepFi_;
    }
    const uint32_t& getIndRSize() const {
        return indRSize_;
    }
    const uint32_t& getIndThetaSize(uint32_t _i = 0) const {
        return indThetaSize_;
    }
    const uint32_t& getIndFiSize(uint32_t _i = 0) const {
        return indFiSize_;
    }
    bool getIdInGridByCoord(const Vector3& _coord, uint32_t& _id) const;
    bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _id) const;
    bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t& _id) const;
    bool getIndInGridById(const uint32_t _id, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getIndInGridByCoord(const Vector3& _coord, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getPolarByCoord(const Vector3& _coord, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getPolarByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getPolarById(const uint32_t _id, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getCoordByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3& _coord) const;
    bool getCoordByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3& _coord) const;
    bool getCoordById(const uint32_t _id, Vector3& _coord) const;
    bool isCoordInGrid(const Vector3& _coord) const;
    bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const;
};



template<typename T>
class MultlPolarGrid : public PolarGrid<T>
{
protected:
    /* data */
    std::vector<Scalar> stepRs_;
    std::vector<Scalar> stepThetas_;
    std::vector<Scalar> stepFis_;
    uint32_t indRSize_;
    std::vector<uint32_t> indThetaSizes_;
    std::vector<uint32_t> indFiSizes_;

public:
    MultlPolarGrid() {};
    ~MultlPolarGrid() {};

    // 通过 PolarGrid 继承
    const Scalar& getStepR(uint32_t _i = 0) const {
        return stepRs_[_i];
    }
    const Scalar& getStepTheta(uint32_t _i = 0) const {
        return stepThetas_[_i];
    }
    const Scalar& getStepFi(uint32_t _i = 0) const {
        return stepFis_[_i];
    }
    const uint32_t& getIndRSize() const {
        return indRSize_;
    }
    const uint32_t& getIndThetaSize(uint32_t _i = 0) const {
        return indThetaSizes_[_i];
    }
    const uint32_t& getIndFiSize(uint32_t _i = 0) const {
        return indFiSizes_[_i];
    }
    bool getIdInGridByCoord(const Vector3& _coord, uint32_t& _id) const;
    bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _id) const;
    bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t& _id) const;
    bool getIndInGridById(const uint32_t _id, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getIndInGridByCoord(const Vector3& _coord, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const;
    bool getPolarByCoord(const Vector3& _coord, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getPolarByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getPolarById(const uint32_t _id, Scalar& _r, Scalar& _theta, Scalar& _fi) const;
    bool getCoordByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3& _coord) const;
    bool getCoordByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3& _coord) const;
    bool getCoordById(const uint32_t _id, Vector3& _coord) const;
    bool isCoordInGrid(const Vector3& _coord) const;
    bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const {
        if (_rInd >= indRSize_ || _thetaInd >= indThetaSizes_[_rInd] || _fiInd >= indFiSizes_[_rInd])
            return false;
        return true;
    }
};

NSP_SLAM_LYJ_END

#endif // SLAM_LYJ_POLARGRID_H