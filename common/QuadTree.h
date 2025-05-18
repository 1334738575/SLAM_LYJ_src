#ifndef SLAM_LYJ_QUADTREE_H
#define SLAM_LYJ_QUADTREE_H

#include "base/base.h"


NSP_SLAM_LYJ_MATH_BEGIN


/// <summary>
///	leftup x<=0 y>0, rightup x>0 y>=0, rightdown x>=0 y<0, leftdown x<0 y<=0 
/// </summary>
/// <typeparam name="T"></typeparam>
template<typename T>
class QuadTree
{
public:
	typedef Eigen::Matrix<T, 2, 1> Vector2;

	struct Node
	{
		Vector2 c;
		bool isLeaf = false;
		Node* lu = nullptr;
		Node* ru = nullptr;
		Node* ld = nullptr;
		Node* rd = nullptr;
		//std::vector<int> ids;

		void build(const std::vector<Vector2>& _vs, const std::vector<int>& _ids,
			const Vector2& _ld, const Vector2& _ru) {
			if (_vs.empty() || _ids.empty())
				return;
			c = (_ld + _ru) / 2;
			//ids = _ids;
			int len = _ids.size();
			if (len == 1) {
				isLeaf = true;
				return;
			}
			std::vector<int> luVs;
			luVs.reserve(len);
			std::vector<int> ruVs;
			ruVs.reserve(len);
			std::vector<int> rdVs;
			rdVs.reserve(len);
			std::vector<int> ldVs;
			ldVs.reserve(len);
			for (int i = 0; i < len; ++i)
			{
				if (_vs[_ids[i]](0) <= c(0) && _vs[_ids[i]](1) > c(1))
					luVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) > c(0) && _vs[_ids[i]](1) >= c(1))
					ruVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) >= c(0) && _vs[_ids[i]](1) < c(1))
					rdVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) < c(0) && _vs[_ids[i]](1) <= c(1))
					ldVs.push_back(_ids[i]);
			}
			//Vector2 _lu(_ld(0), _ru(1));
			//Vector2 _dr(_ru(0), _ld(1));
			if (!luVs.empty()) {
				lu = new Node();
				lu->build(_vs, luVs, Vector2(_ld(0), c(1)), Vector2(c(0), _ru(1)));
			}
			if (!ruVs.empty()) {
				ru = new Node();
				ru->build(_vs, ruVs, c, _ru);
			}
			if (!rdVs.empty()) {
				rd = new Node();
				rd->build(_vs, rdVs, Vector2(c(0), _ld(1)), Vector2(_ru(0), c(1)));
			}
			if (!ldVs.empty()) {
				ld = new Node();
				ld->build(_vs, ldVs, _ld, c);
			}
		}

		void release() {
			//vs.swap(std::vector<V*>());
			//if (isLeaf) {
			//	vs.swap(std::vector<V>());
			//	return;
			//}
			if (lu) { 
				lu->release(); 
				delete lu;
			}
			if (ru) {
				ru->release();
				delete ru;
			}
			if (ld) {
				ld->release();
				delete ld;
			}
			if (rd) {
				rd->release();
				delete rd;
			}
		}

		bool hit(const Vector2& _v) {
			if (_v(0) <= c(0) && _v(1) > c(1)) {
				if (lu)
					return lu->hit(_v);
				else
					return false;
			}
			else if (_v(0) > c(0) && _v(1) >= c(1)) {
				if (ru)
					return ru->hit(_v);
				else
					return false;
			}
			else if (_v(0) >= c(0) && _v(1) < c(1)) {
				if (rd)
					return rd->hit(_v);
				else
					return false;
			}
			else if (_v(0) < c(0) && _v <= c(1)) {
				if (ld)
					return ld->hit(_v);
				else
					return false;
			}
		}
	};

	QuadTree() {};
	~QuadTree() {};

	void build(const std::vector<Vector2>& _vs) {
		if (root) {
			root->release();
			delete root;
			root = nullptr;
		}
		if (_vs.empty())
			return;
		Vector2 ld = _vs[0];
		Vector2 ru = _vs[0];
		int len = _vs.size();
		std::vector<int> ids(len, 0);
		for (int i = 1; i < len; ++i)
		{
			ids[i] = i;
			if (ld(0) > _vs[i](0))
				ld(0) = _vs[i](0);
			else if (ru(0) < _vs[i](0))
				ru(0) = _vs[i](0);
			if (ld(1) > _vs[i](1))
				ld(1) = _vs[i](1);
			else if (ru(1) < _vs[i](1))
				ru(1) = _vs[i](1);
		}
		root = new Node();
		root->build(_vs, ids, ld, ru);
	}

	void release() {
		if (root) {
			root->release();
			delete root;
		}
	}

	bool hit(const Vector2& _v) {
		if (root)
			return root->hit(_v);
		return false;
	}

private:
	//std::vector<Vector2> vs;
	Node* root = nullptr;
};

using QuadTreed = QuadTree<double>;
using QuadTreef = QuadTree<float>;


NSP_SLAM_LYJ_MATH_END


#endif // !SLAM_LYJ_QUADTREE_H
