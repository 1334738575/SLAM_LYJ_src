#ifndef SLAM_LYJ_POLARGRID_H
#define SLAM_LYJ_POLARGRID_H

#include "base/Base.h"
#include "CommonAlgorithm.h"

#ifndef PI
#define PI 3.14159265358979323846264338327950288419
#endif
#ifndef TWICEPI
#define TWICEPI 2 * PI
#endif

NSP_SLAM_LYJ_MATH_BEGIN
/// <summary>
/// 极坐标网格，内部会默认扩展一步长
/// </summary>
/// <typeparam name="T"></typeparam>
template <typename T>
class PolarGrid
{
protected:
    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

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
    uint32_t indRSize_ = 0;

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

    virtual const Scalar &getStepR(uint32_t _i = 0) const = 0;
    virtual const Scalar &getStepTheta(uint32_t _i = 0) const = 0;
    virtual const Scalar &getStepFi(uint32_t _i = 0) const = 0;
    virtual const uint32_t &getIndRSize() const
    {
        return indRSize_;
    }
    virtual const uint32_t &getIndThetaSize(uint32_t _i = 0) const = 0;
    virtual const uint32_t &getIndFiSize(uint32_t _i = 0) const = 0;
    virtual bool getIdInGridByCoord(const Vector3 &_coord, uint32_t &_id) const
    {
        _id = -1;
        Scalar r = 0;
        Scalar theta = 0;
        Scalar fi = 0;
        this->convertCoord2Ploar(_coord, r, theta, fi);
        return getIdInGridByPolar(r, theta, fi, _id);
    }
    virtual bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_id) const
    {
        uint32_t indR, indTheta, indFi;
        return getIndInGridByPolar(_r, _theta, _fi, indR, indTheta, indFi) && getIdInGridByInd(indR, indTheta, indFi, _id);
    }
    virtual bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t &_id) const = 0;
    virtual bool getIndInGridById(uint32_t _id, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getIndInGridByCoord(const Vector3 &_coord, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const
    {
        float r = 0;
        float theta = 0;
        float fi = 0;
        this->convertCoord2Ploar(_coord, r, theta, fi);
        return getIndInGridByPolar(r, theta, fi, _rInd, _thetaInd, _fiInd);
    }
    virtual bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const = 0;
    virtual bool getPolarInGridByCoord(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi) const
    {
        convertCoord2Ploar(_coord, _r, _theta, _fi);
        return isPolarInGrid(_r, _theta, _fi);
    }
    virtual bool getPolarInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar &_r, Scalar &_theta, Scalar &_fi) const = 0;
    virtual bool getPolarInGridById(const uint32_t _id, Scalar &_r, Scalar &_theta, Scalar &_fi) const
    {
        uint32_t indR, indTheta, indFi;
        return getIndInGridById(_id, indR, indTheta, indFi) && getPolarInGridByInd(indR, indTheta, indFi, _r, _theta, _fi);
    }
    virtual bool getCoordInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord) const
    {
        if (!isPolarInGrid(_r, _theta, _fi))
            return false;
        convertPolar2Coord(_r, _theta, _fi, _coord);
        return true;
    }
    virtual bool getCoordInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3 &_coord) const
    {
        Scalar r = 0;
        Scalar theta = 0;
        Scalar fi = 0;
        return getPolarInGridByInd(_rInd, _thetaInd, _fiInd, r, theta, fi) && getCoordInGridByPolar(r, theta, fi, _coord);
    }
    virtual bool getCoordInGridById(const uint32_t _id, Vector3 &_coord) const
    {
        uint32_t indR, indTheta, indFi;
        return getIndInGridById(_id, indR, indTheta, indFi) && getCoordInGridByInd(indR, indTheta, indFi, _coord);
    }
    virtual bool isCoordInGrid(const Vector3 &_coord) const
    {
        float r = 0;
        float theta = 0;
        float fi = 0;
        convertCoord2Ploar(_coord, r, theta, fi);
        return isPolarInGrid(r, theta, fi);
    }
    virtual bool isPolarInGrid(const Scalar _r, const Scalar _theta, const Scalar _fi) const
    {
        if (_r > maxR_ || _r < minR_ || _theta > maxTheta_ || _theta < minTheta_ || _fi > maxFi_ || _fi < minFi_)
            return false;
        return true;
    }
    virtual bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const = 0;
    virtual bool isIdInGrid(const uint32_t _id) const
    {
        if (_id >= totalSize_)
            return false;
        return true;
    }
    virtual bool getPoseInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, Matrix3 &_rot, Vector3 &_trans) const
    {
        if (!isPolarInGrid(_r, _theta, _fi))
            return false;
        convertPolar2Pose(_r, _theta, _fi, _rot, _trans);
        return true;
    }
    virtual bool getPoseInGridByCoord(const Vector3 &_coord, Matrix3 &_rot, Vector3 &_trans) const
    {
        if (!isCoordInGrid(_coord))
            return false;
        convertPolar2Pose(_coord, _rot, _trans);
        return true;
    }
    virtual bool getPoseInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Matrix3 &_rot, Vector3 &_trans) const
    {
        Scalar r, theta, fi;
        if (!getPolarInGridByInd(_rInd, _thetaInd, _fiInd, r, theta, fi))
            return false;
        convertPolar2Pose(r, theta, fi, _rot, _trans);
        return true;
    }
    virtual bool getPoseInGridById(const uint32_t &_id, Matrix3 &_rot, Vector3 &_trans) const
    {
        Scalar r, theta, fi;
        if (!getPolarInGridById(_id, r, theta, fi))
            return false;
        convertPolar2Pose(r, theta, fi, _rot, _trans);
        return true;
    }
    virtual void getAroundIdByCoord(const Vector3 &_coord, uint32_t _level, std::vector<uint32_t> &_ids) const = 0;

    static void convertCoord2Ploar(const Vector3 &_coord, Scalar &_r, Scalar &_theta, Scalar &_fi)
    {
        _r = _coord.norm();
        if (_r < 1e-5)
        {
            _r = 0.0f;
            _theta = 0.0f;
            _fi = 0.0f;
            return;
        }
        _theta = acosf(_coord(2) / _r);
        _fi = atan2f(_coord(1), _coord(0));
    };
    static void convertPolar2Coord(const Scalar _r, const Scalar _theta, const Scalar _fi, Vector3 &_coord)
    {
        _coord(0) = _r * sin(_theta) * cos(_fi);
        _coord(1) = _r * sin(_theta) * sin(_fi);
        _coord(2) = _r * cos(_theta);
    };
    /// <summary>
    /// local voxel to grid
    /// </summary>
    /// <param name="_r"></param>
    /// <param name="_theta"></param>
    /// <param name="_fi"></param>
    /// <param name="_rot"></param>
    /// <param name="_trans"></param>
    static void convertPolar2Pose(const Scalar _r, const Scalar _theta, const Scalar _fi, Matrix3 &_rot, Vector3 &_trans)
    {
        convertPolar2Coord(_r, _theta, _fi, _trans);
        Scalar invFi = _fi + PI;
        Vector3 axisFi(0, 0, 1);
        Matrix3 rotFi = Rodrigues2RotMatrix<Scalar>(axisFi, invFi);
        Scalar invTheta = PI - _theta;
        Vector3 axisTheta(0, 1, 0);
        Matrix3 rotTheta = Rodrigues2RotMatrix<Scalar>(axisTheta, invTheta);
        _rot = rotFi * rotTheta;
    }
    static void convertPolar2Pose(const Vector3 &_coord, Matrix3 &_rot, Vector3 &_trans)
    {
        _trans = _coord;
        Scalar r, theta, fi;
        convertCoord2Ploar(_coord, r, theta, fi);
        Scalar invFi = fi + PI;
        Vector3 axisFi(0, 0, 1);
        Matrix3 rotFi = Rodrigues2RotMatrix<Scalar>(axisFi, invFi);
        Scalar invTheta = PI - theta;
        Vector3 axisTheta(0, 1, 0);
        Matrix3 rotTheta = Rodrigues2RotMatrix<Scalar>(axisTheta, invTheta);
        _rot = rotFi * rotTheta;
    }
};

template <typename T>
class SoloPolarGrid : public PolarGrid<T>
{
protected:
    /* data */
    Scalar stepR_ = 0;
    Scalar stepTheta_ = 0.1;
    Scalar stepFi_ = 0.1;
    // uint32_t indRSize_ = 0;
    uint32_t indThetaSize_ = 0;
    uint32_t indFiSize_ = 0;

    static std::vector<Eigen::Vector3i> nearLocs_;

public:
    SoloPolarGrid(Scalar _maxR,
                  Scalar _stepR, Scalar _stepTheta, Scalar _stepFi,
                  const T &_initData,
                  Scalar _maxTheta = PI, Scalar _maxFi = TWICEPI,
                  Scalar _minR = 0.f, Scalar _minTheta = 0.f, Scalar _minFi = 0.f)
    {
        minR_ = _minR;
        minTheta_ = _minTheta;
        minFi_ = _minFi;
        stepR_ = _stepR;
        stepTheta_ = _stepTheta;
        stepFi_ = _stepFi;
        maxR_ = _maxR + _stepR;
        maxTheta_ = _maxTheta;
        maxFi_ = _maxFi;
        indRSize_ = (maxR_ - minR_) / stepR_;
        indThetaSize_ = (maxTheta_ - minTheta_) / stepTheta_;
        indFiSize_ = (maxFi_ - minFi_) / stepFi_;
        totalSize_ = indRSize_ * indThetaSize_ * indFiSize_;
        datas_.resize(totalSize_);
        for (auto &data : datas_)
            data = _initData;
    };
    ~SoloPolarGrid() {};

    // 通过 PolarGrid 继承
    const Scalar &getStepR(uint32_t _i = 0) const override
    {
        return stepR_;
    }
    const Scalar &getStepTheta(uint32_t _i = 0) const override
    {
        return stepTheta_;
    }
    const Scalar &getStepFi(uint32_t _i = 0) const override
    {
        return stepFi_;
    }
    // const uint32_t& getIndRSize() const override {
    //     return indRSize_;
    // }
    const uint32_t &getIndThetaSize(uint32_t _i = 0) const override
    {
        return indThetaSize_;
    }
    const uint32_t &getIndFiSize(uint32_t _i = 0) const override
    {
        return indFiSize_;
    }
    // bool getIdInGridByCoord(const Vector3& _coord, uint32_t& _id) const override;
    // bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _id) const override;
    bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t &_id) const override
    {
        if (!this->isIndInGrid(_rInd, _thetaInd, _fiInd))
            return false;
        _id = 0;
        _id += (_rInd * indThetaSize_ * indFiSize_);
        _id += indFiSize_ * _thetaInd;
        _id += _fiInd;
        return true;
    }
    bool getIndInGridById(uint32_t _id, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const override
    {
        if (!this->isIdInGrid(_id))
            return false;
        uint32_t idTmp = _id;
        _rInd = _id / (this->indThetaSize_ * this->indFiSize_);
        if (_rInd >= indRSize_)
            return false;
        _id -= (_rInd * this->indThetaSize_ * this->indFiSize_);
        _thetaInd = _id / this->indFiSize_;
        _fiInd = _id - _thetaInd * this->indFiSize_;
        return true;
    }
    // bool getIndInGridByCoord(const Vector3& _coord, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const override;
    bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const override
    {
        return this->getIndRInGrid(_r, _rInd) && this->getIndThetaInGrid(_theta, _thetaInd) && this->getIndFiInGrid(_fi, _fiInd);
    }
    bool getPolarInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar &_r, Scalar &_theta, Scalar &_fi) const override
    {
        return isIndInGrid(_rInd, _thetaInd, _fiInd) && this->getRInGrid(_rInd, _r) && this->getThetaInGrid(_thetaInd, _theta) && this->getFiInGrid(_fiInd, _fi);
    }
    // bool getPolarInGridById(const uint32_t _id, Scalar& _r, Scalar& _theta, Scalar& _fi) const override;
    // bool getCoordInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3& _coord) const override;
    // bool getCoordInGridById(const uint32_t _id, Vector3& _coord) const override;
    bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const override
    {
        if (_rInd >= indRSize_ || _thetaInd >= indThetaSize_ || _fiInd >= indFiSize_)
            return false;
        return true;
    }
    // 使用了int，后续可能会有溢出问题
    void getAroundIdByCoord(const Vector3 &_coord, uint32_t _level, std::vector<uint32_t> &_ids) const
    {
        _ids.clear();
        uint32_t indR, indTheta, indFi;
        if (!getIndInGridByCoord(_coord, indR, indTheta, indFi))
            return;
        // buffer
        std::queue<Eigen::Vector3i> inds;
        std::unordered_set<uint32_t> ids;
        uint32_t id;
        uint32_t size = 0;
        uint32_t i = 0;
        uint32_t j = 0;
        Eigen::Vector3i ind;
        Eigen::Vector3i ind2;
        getIdInGridByInd(indR, indTheta, indFi, id);
        ids.insert(id);
        inds.push(Eigen::Vector3i(indR, indTheta, indFi));
        while (_level > 0)
        {
            size = inds.size();
            for (i = 0; i < size; ++i)
            {
                ind = inds.front();
                inds.pop();
                for (j = 0; j < nearLocs_.size(); ++j)
                {
                    ind2 = ind + nearLocs_[j];
                    if (!getIdInGridByInd(ind2(0), ind2(1), ind2(2), id) || ids.count(id))
                        continue;
                    ids.insert(id);
                    inds.push(ind2);
                }
            }
            --_level;
        }
        _ids.reserve(ids.size());
        for (auto &id : ids)
            _ids.push_back(id);
    }

protected:
    bool getIndRInGrid(Scalar _r, uint32_t &_indR) const
    {
        if (_r < this->minR_ || _r > this->maxR_)
            return false;
        _indR = (_r - minR_) / stepR_;
        return true;
    }
    bool getIndThetaInGrid(const Scalar _theta, uint32_t &_indTheta) const
    {
        if (_theta < this->minTheta_ || _theta > this->maxTheta_)
            return false;
        _indTheta = (_theta - minTheta_) / stepTheta_;
        return true;
    }
    bool getIndFiInGrid(const Scalar _fi, uint32_t &_indFi) const
    {
        if (_fi < this->minFi_ || _fi > this->maxFi_)
            return false;
        _indFi = (_fi - minFi_) / stepFi_;
        return true;
    }
    bool getRInGrid(const uint32_t _indR, Scalar &_r) const
    {
        if (_indR < 0 || _indR >= this->indRSize_)
            return false;
        _r = this->minR_;
        _r += _indR * stepR_;
        _r += (stepR_ / 2);
        return true;
    }
    bool getThetaInGrid(const uint32_t _indTheta, Scalar &_theta) const
    {
        if (_indTheta < 0 || _indTheta >= this->indThetaSize_)
            return false;
        _theta = this->minTheta_;
        _theta += stepTheta_ * _indTheta;
        _theta += (stepTheta_ / 2);
        return true;
    }
    bool getFiInGrid(const uint32_t _indFi, Scalar &_fi) const
    {
        if (_indFi < 0 || _indFi >= this->indFiSize_)
            return false;
        _fi = this->minFi_;
        _fi += stepFi_ * _indFi;
        _fi += (stepFi_ / 2);
        return true;
    }
};
template <typename T>
std::vector<Eigen::Vector3i> SoloPolarGrid<T>::nearLocs_ = {
    Eigen::Vector3i(-1, 0, 0),
    Eigen::Vector3i(1, 0, 0),
    Eigen::Vector3i(0, -1, 0),
    Eigen::Vector3i(0, 1, 0),
    Eigen::Vector3i(0, 0, -1),
    Eigen::Vector3i(0, 0, 1),
};

template <typename T>
class MultiPolarGrid : public PolarGrid<T>
{
protected:
    /* data */
    std::vector<Scalar> stepRs_;
    std::vector<Scalar> stepThetas_;
    std::vector<Scalar> stepFis_;
    // uint32_t indRSize_;
    std::vector<uint32_t> indThetaSizes_;
    std::vector<uint32_t> indFiSizes_;

    static std::vector<Eigen::Vector3i> nearLocs_;

public:
    MultiPolarGrid(
        std::vector<Scalar> _stepRs, Scalar _stepL,
        const T &_initData,
        Scalar _maxTheta = PI, Scalar _maxFi = TWICEPI,
        Scalar _minR = 0.f, Scalar _minTheta = 0.f, Scalar _minFi = 0.f)
    {
        uint32_t RSize = _stepRs.size();
        minR_ = _minR;
        minTheta_ = _minTheta;
        minFi_ = _minFi;

        maxR_ = _minR + _stepRs.back();
        for (int i = 0; i < RSize; ++i)
            maxR_ += _stepRs[i];
        maxTheta_ = _maxTheta;
        maxFi_ = _maxFi;

        stepRs_ = _stepRs;
        stepThetas_.resize(_stepRs.size());
        stepFis_.resize(_stepRs.size());
        Scalar tmpR = _minR;
        for (int i = 0; i < RSize; ++i)
        {
            tmpR += _stepRs[i];
            // L / r = 2 * PI;
            stepThetas_[i] = _stepL / tmpR;
            stepFis_[i] = stepThetas_[i];
        }

        indRSize_ = RSize;
        indThetaSizes_.resize(RSize);
        indFiSizes_.resize(RSize);
        for (int i = 0; i < RSize; ++i)
        {
            indThetaSizes_[i] = (_maxTheta - _minTheta) / stepThetas_[i] + 1;
            indFiSizes_[i] = (_maxFi - _minFi) / stepFis_[i] + 1;
        }

        totalSize_ = 0;
        for (int i = 0; i < RSize; ++i)
            totalSize_ += (indThetaSizes_[i] * indFiSizes_[i]);

        datas_.resize(totalSize_);
        for (auto &data : datas_)
            data = _initData;
    };
    ~MultiPolarGrid() {};

    // 通过 PolarGrid 继承
    const Scalar &getStepR(uint32_t _i = 0) const override
    {
        return stepRs_[_i];
    }
    const Scalar &getStepTheta(uint32_t _i = 0) const override
    {
        return stepThetas_[_i];
    }
    const Scalar &getStepFi(uint32_t _i = 0) const override
    {
        return stepFis_[_i];
    }
    // const uint32_t& getIndRSize() const override {
    //     return indRSize_;
    // }
    const uint32_t &getIndThetaSize(uint32_t _i = 0) const override
    {
        return indThetaSizes_[_i];
    }
    const uint32_t &getIndFiSize(uint32_t _i = 0) const override
    {
        return indFiSizes_[_i];
    }
    // bool getIdInGridByCoord(const Vector3& _coord, uint32_t& _id) const override {
    //     _id = -1;
    //     Scalar r = 0;
    //     Scalar theta = 0;
    //     Scalar fi = 0;
    //     this->convertCoord2Ploar(_coord, r, theta, fi);
    //     return getIdInGridByPolar(r, theta, fi, _id);
    // }
    // bool getIdInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t& _id) const override {
    //     uint32_t indR, indTheta, indFi;
    //     return getIndInGridByPolar(_r, _theta, _fi, indR, indTheta, indFi) && getIdInGridByInd(indR, indTheta, indFi, _id);
    // }
    bool getIdInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, uint32_t &_id) const override
    {
        if (!this->isIndInGrid(_rInd, _thetaInd, _fiInd))
            return false;
        _id = 0;
        for (int i = 0; i < _rInd; ++i)
            _id += (indThetaSizes_[i] * indFiSizes_[i]);
        _id += indFiSizes_[_rInd] * _thetaInd;
        _id += _fiInd;
        return true;
    }
    bool getIndInGridById(uint32_t _id, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const override
    {
        if (!this->isIdInGrid(_id))
            return false;
        uint32_t idTmp = _id;
        uint32_t s;
        for (uint32_t i = 0; i < indRSize_; ++i)
        {
            s = this->indThetaSizes_[i] * this->indFiSizes_[i];
            if (_id < s)
            {
                _rInd = i;
                _thetaInd = idTmp / this->indFiSizes_[i];
                _fiInd = idTmp - _thetaInd * this->indFiSizes_[i];
                return true;
            }
            _id -= s;
            idTmp = _id;
        }
        return false;
    }
    // bool getIndInGridByCoord(const Vector3& _coord, uint32_t& _rInd, uint32_t& _thetaInd, uint32_t& _fiInd) const override {
    //     float r = 0;
    //     float theta = 0;
    //     float fi = 0;
    //     this->convertCoord2Ploar(_coord, r, theta, fi);
    //     return getIndInGridByPolar(r, theta, fi, _rInd, _thetaInd, _fiInd);
    // }
    bool getIndInGridByPolar(const Scalar _r, const Scalar _theta, const Scalar _fi, uint32_t &_rInd, uint32_t &_thetaInd, uint32_t &_fiInd) const override
    {
        return this->getIndRInGrid(_r, _rInd) && this->getIndThetaInGrid(_rInd, _theta, _thetaInd) && this->getIndFiInGrid(_rInd, _fi, _fiInd);
    }
    bool getPolarInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Scalar &_r, Scalar &_theta, Scalar &_fi) const override
    {
        return isIndInGrid(_rInd, _thetaInd, _fiInd) && this->getRInGrid(_rInd, _r) && this->getThetaInGrid(_rInd, _thetaInd, _theta) && this->getFiInGrid(_rInd, _fiInd, _fi);
    }
    // bool getPolarInGridById(const uint32_t _id, Scalar& _r, Scalar& _theta, Scalar& _fi) const override {
    //     uint32_t indR, indTheta, indFi;
    //     return getIndInGridById(_id, indR, indTheta, indFi) && getPolarInGridByInd(indR, indTheta, indFi, _r, _theta, _fi);
    // }
    // bool getCoordInGridByInd(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd, Vector3& _coord) const override {
    //     Scalar r = 0;
    //     Scalar theta = 0;
    //     Scalar fi = 0;
    //     return getPolarInGridByInd(_rInd, _thetaInd, _fiInd, r, theta, fi) && getCoordInGridByPolar(r, theta, fi, _coord);
    // }
    // bool getCoordInGridById(const uint32_t _id, Vector3& _coord) const override {
    //     uint32_t indR, indTheta, indFi;
    //     return getIndInGridById(_id, indR, indTheta, indFi) && getCoordInGridByInd(indR, indTheta, indFi, _coord);
    // }
    bool isIndInGrid(const uint32_t _rInd, const uint32_t _thetaInd, const uint32_t _fiInd) const override
    {
        if (_rInd >= indRSize_ || _thetaInd >= indThetaSizes_[_rInd] || _fiInd >= indFiSizes_[_rInd])
            return false;
        return true;
    }
    void getAroundIdByCoord(const Vector3 &_coord, uint32_t _level, std::vector<uint32_t> &_ids) const
    {
        _ids.clear();
        uint32_t indR, indTheta, indFi;
        if (!getIndInGridByCoord(_coord, indR, indTheta, indFi))
            return;
        // buffer
        std::queue<Eigen::Vector3i> inds;
        std::unordered_set<uint32_t> ids;
        uint32_t id;
        uint32_t size = 0;
        uint32_t i = 0;
        uint32_t j = 0;
        Eigen::Vector3i ind;
        Eigen::Vector3i ind2;
        Scalar r, theta, fi;
        Scalar r2, theta2, fi2;
        getIdInGridByInd(indR, indTheta, indFi, id);
        ids.insert(id);
        inds.push(Eigen::Vector3i(indR, indTheta, indFi));
        while (_level > 0)
        {
            size = inds.size();
            for (i = 0; i < size; ++i)
            {
                ind = inds.front();
                inds.pop();
                for (j = 0; j < nearLocs_.size(); ++j)
                {
                    ind2 = ind + nearLocs_[j];
                    if (!getIdInGridByInd(ind2(0), ind2(1), ind2(2), id) || ids.count(id))
                        continue;
                    ids.insert(id);
                    inds.push(ind2);
                }
                getPolarInGridByInd(ind(0), ind(1), ind(2), r, theta, fi);
                r2 = r;
                theta2 = theta;
                fi2 = fi;
                if (ind(0) + 1 <= stepRs_.size())
                {
                    r2 += stepRs_[ind(0)];
                    getIndInGridByPolar(r2, theta2, fi2, indR, indTheta, indFi);
                    if (!getIdInGridByInd(indR, indTheta, indFi, id) || ids.count(id))
                        continue;
                    ids.insert(id);
                    inds.push(Eigen::Vector3i(indR, indTheta, indFi));
                }
                r2 = r;
                theta2 = theta;
                fi2 = fi;
                if (ind(0) - 1 >= 0)
                {
                    r2 -= stepRs_[ind(0) - 1];
                    getIndInGridByPolar(r2, theta2, fi2, indR, indTheta, indFi);
                    if (!getIdInGridByInd(indR, indTheta, indFi, id) || ids.count(id))
                        continue;
                    ids.insert(id);
                    inds.push(Eigen::Vector3i(indR, indTheta, indFi));
                }
            }
            --_level;
        }
        _ids.reserve(ids.size());
        for (auto &id : ids)
            _ids.push_back(id);
    }

protected:
    bool getIndRInGrid(Scalar _r, uint32_t &_indR) const
    {
        if (_r < this->minR_ || _r > this->maxR_)
            return false;
        _r -= minR_;
        for (uint32_t i = 0; i < indRSize_; ++i)
        {
            _r -= stepRs_[i];
            if (_r < 0)
            {
                _indR = i;
                return true;
            }
        }
        return false;
    }
    bool getIndThetaInGrid(const uint32_t _indR, const Scalar _theta, uint32_t &_indTheta) const
    {
        if (_theta < this->minTheta_ || _theta > this->maxTheta_)
            return false;
        const Scalar &stepTheta = this->stepThetas_[_indR];
        _indTheta = (_theta - minTheta_) / stepTheta;
        return true;
    }
    bool getIndFiInGrid(const uint32_t _indR, const Scalar _fi, uint32_t &_indFi) const
    {
        if (_fi < this->minFi_ || _fi > this->maxFi_)
            return false;
        const Scalar &stepFi = this->stepFis_[_indR];
        _indFi = (_fi - minFi_) / stepFi;
        return true;
    }
    bool getRInGrid(const uint32_t _indR, Scalar &_r) const
    {
        if (_indR < 0 || _indR >= this->indRSize_)
            return false;
        _r = this->minR_;
        for (uint32_t i = 0; i < _indR; ++i)
            _r += stepRs_[i];
        _r += (stepRs_[_indR] / 2);
        return true;
    }
    bool getThetaInGrid(const uint32_t _indR, const uint32_t _indTheta, Scalar &_theta) const
    {
        if (_indTheta < 0 || _indTheta >= this->indThetaSizes_[_indR])
            return false;
        _theta = this->minTheta_;
        _theta += stepThetas_[_indR] * _indTheta;
        _theta += (stepThetas_[_indR] / 2);
        return true;
    }
    bool getFiInGrid(const uint32_t _indR, const uint32_t _indFi, Scalar &_fi) const
    {
        if (_indFi < 0 || _indFi >= this->indFiSizes_[_indR])
            return false;
        _fi = this->minFi_;
        _fi += stepFis_[_indR] * _indFi;
        _fi += (stepFis_[_indR] / 2);
        return true;
    }
};
template <typename T>
std::vector<Eigen::Vector3i> MultiPolarGrid<T>::nearLocs_ = {
    Eigen::Vector3i(0, -1, 0),
    Eigen::Vector3i(0, 1, 0),
    Eigen::Vector3i(0, 0, -1),
    Eigen::Vector3i(0, 0, 1),
};

NSP_SLAM_LYJ_MATH_END

#endif // SLAM_LYJ_POLARGRID_H