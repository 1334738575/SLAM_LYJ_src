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
    inline const uint32_t &getMaxR() const { return totalSize_; };
    inline const std::vector<T> &getMaxR() const { return datas_; };

    virtual const Scalar &getStepRSize(uint32_t _i) const = 0;
    virtual const Scalar &getStepThetaSize(uint32_t _i) const = 0;
    virtual const Scalar &getStepFiSize(uint32_t _i) const = 0;
    virtual const uint32_t &getIndRSize(uint32_t _i) const = 0;
    virtual const uint32_t &getIndThetaSize(uint32_t _i) const = 0;
    virtual const uint32_t &getIndFiSize(uint32_t _i) const = 0;

    virtual bool getIdInGridByCoord(const Vector3 &_coord, uint32_t &_id) const = 0;
    virtual bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_id) const = 0;
    virtual bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t &_id) const = 0;
    virtual bool getIndInGridById(const uint32_t _id, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getIndInGridByCoord(const Vector3 &_coord, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) = 0;
    virtual bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getPolarByCoord(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getPolarByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getPolarById(const uint32_t _id, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getCoordByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord) const = 0;
    virtual bool getCoordByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3 &_coord) const = 0;
    virtual bool getCoordById(const uint32_t _id, Vector3 &_coord) const = 0;
    virtual bool isCoordInGrid(const Vector3 &_coord) const = 0;
    virtual bool isPolarInGrid(const Scalar _r, const Scalar _theta, const Scalar _fi) const = 0;
    virtual bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const = 0;
    virtual bool isIdInGrid(const uint32_t _id) const = 0;
    static void convertCoord2Ploar(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi) {

    };
    static void convertPolar2Coord(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord) {

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

public:
    SoloPolarGrid(/* args */);
    ~SoloPolarGrid();
};

template <typename T>
SoloPolarGrid<T>::SoloPolarGrid(/* args */)
{
}

template <typename T>
SoloPolarGrid<T>::~SoloPolarGrid()
{
}

NSP_SLAM_LYJ_END

#endif // SLAM_LYJ_POLARGRID_H