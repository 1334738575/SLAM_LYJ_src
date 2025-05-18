#ifndef SLAM_LYJ_OCTREE_H
#define SLAM_LYJ_OCTREE_H

#include "base/base.h"

NSP_SLAM_LYJ_MATH_BEGIN

/// <summary>
///	leftfrontup x<0 y>0 z>0,
/// rightfrontup x>=0 y>=0 z>=0,
/// rightbackup x>0 y<0 z>0,
/// leftbackup x<0 y<0 z>0,
/// leftfrontdown x<0 y>0 z<0,
/// rightfrontdown x>0 y>0 z<0,
/// rightbackdown x>0 y<0 z<0,
/// leftbackdown x<=0 y<=0 z<=0
/// </summary>
/// <typeparam name="T"></typeparam>
template <typename T>
class OcTree
{
public:
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	struct Node
	{
		Vector3 c;
		bool isLeaf = false;
		Node *lfu = nullptr;
		Node *rfu = nullptr;
		Node *rbu = nullptr;
		Node *lbu = nullptr;
		Node *lfd = nullptr;
		Node *rfd = nullptr;
		Node *rbd = nullptr;
		Node *lbd = nullptr;
		std::vector<int> ids;
		Vector3 lfuP;
		Vector3 rfuP;
		Vector3 rbuP;
		Vector3 lbuP;
		Vector3 lfdP;
		Vector3 rfdP;
		Vector3 rbdP;
		Vector3 lbdP;

		Node()
		{
		}
		Node(const std::vector<Vector3>& _vs, const std::vector<int>& _ids,
			const Vector3& _lbd, const Vector3& _rfu)
			:ids(_ids), lbdP(_lbd), rfuP(_rfu)
		{
			devide(_vs);
		}

		void build(const std::vector<Vector3> &_vs, const std::vector<int> &_ids,
				   const Vector3 &_lbd, const Vector3 &_rfu)
		{
			if (_vs.empty() || _ids.empty())
				return;
			c = (_lbd + _rfu) / 2;
			// ids = _ids;
			int len = _ids.size();
			if (len == 1)
			{
				isLeaf = true;
				ids = _ids;
				return;
			}
			std::vector<int> lfuVs;
			lfuVs.reserve(len);
			std::vector<int> rfuVs;
			rfuVs.reserve(len);
			std::vector<int> rbuVs;
			rbuVs.reserve(len);
			std::vector<int> lbuVs;
			lbuVs.reserve(len);
			std::vector<int> lfdVs;
			lfdVs.reserve(len);
			std::vector<int> rfdVs;
			rfdVs.reserve(len);
			std::vector<int> rbdVs;
			rbdVs.reserve(len);
			std::vector<int> lbdVs;
			lbdVs.reserve(len);
			for (int i = 0; i < len; ++i)
			{
				if (_vs[_ids[i]](0) < c(0) && _vs[_ids[i]](1) > c(1) && _vs[_ids[i]](2) > c(2))
					lfuVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) >= c(0) && _vs[_ids[i]](1) >= c(1) && _vs[_ids[i]](2) >= c(2))
					rfuVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) > c(0) && _vs[_ids[i]](1) < c(1) && _vs[_ids[i]](2) > c(2))
					rbuVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) < c(0) && _vs[_ids[i]](1) < c(1) && _vs[_ids[i]](2) > c(2))
					lbuVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) < c(0) && _vs[_ids[i]](1) > c(1) && _vs[_ids[i]](2) < c(2))
					lfdVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) > c(0) && _vs[_ids[i]](1) > c(1) && _vs[_ids[i]](2) < c(2))
					rfdVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) > c(0) && _vs[_ids[i]](1) < c(1) && _vs[_ids[i]](2) < c(2))
					rbdVs.push_back(_ids[i]);
				else if (_vs[_ids[i]](0) <= c(0) && _vs[_ids[i]](1) <= c(1) && _vs[_ids[i]](2) <= c(2))
					lbdVs.push_back(_ids[i]);
			}
			if (!lfuVs.empty())
			{
				lfu = new Node();
				lfu->build(_vs, lfuVs, Vector3(_lbd(0), c(1), c(2)), Vector3(c(0), _rfu(1), _rfu(2)));
			}
			if (!rfuVs.empty())
			{
				rfu = new Node();
				rfu->build(_vs, rfuVs, c, _rfu);
			}
			if (!rbuVs.empty())
			{
				rbu = new Node();
				rbu->build(_vs, rbuVs, Vector3(c(0), _lbd(1), c(2)), Vector3(_rfu(0), c(1), _rfu(2)));
			}
			if (!lbuVs.empty())
			{
				lbu = new Node();
				lbu->build(_vs, lbuVs, Vector3(_lbd(0), _lbd(1), c(2)), Vector3(c(0), c(1), _rfu(2)));
			}
			if (!lfdVs.empty())
			{
				lfd = new Node();
				lfd->build(_vs, lfdVs, Vector3(_lbd(0), c(1), _lbd(2)), Vector3(c(0), _rfu(1), c(2)));
			}
			if (!rfdVs.empty())
			{
				rfd = new Node();
				rfd->build(_vs, rfdVs, Vector3(c(0), c(1), _lbd(2)), Vector3(_rfu(0), _rfu(1), c(2)));
			}
			if (!rbdVs.empty())
			{
				rbd = new Node();
				rbd->build(_vs, rbdVs, Vector3(c(0), _lbd(1), _lbd(2)), Vector3(_rfu(0), c(1), c(2)));
			}
			if (!lbdVs.empty())
			{
				lbd = new Node();
				lbd->build(_vs, lbdVs, _lbd, c);
			}
		}		
		
		void devide(const std::vector<Vector3>& _vs)
		{
			if (_vs.empty() || ids.empty())
				return;
			c = (lbd + rfu) / 2;
			int len = ids.size();
			if (len == 1)
			{
				isLeaf = true;
				return;
			}
			std::vector<int> lfuVs;
			lfuVs.reserve(len);
			std::vector<int> rfuVs;
			rfuVs.reserve(len);
			std::vector<int> rbuVs;
			rbuVs.reserve(len);
			std::vector<int> lbuVs;
			lbuVs.reserve(len);
			std::vector<int> lfdVs;
			lfdVs.reserve(len);
			std::vector<int> rfdVs;
			rfdVs.reserve(len);
			std::vector<int> rbdVs;
			rbdVs.reserve(len);
			std::vector<int> lbdVs;
			lbdVs.reserve(len);
			for (int i = 0; i < len; ++i)
			{
				if (_vs[ids[i]](0) < c(0) && _vs[ids[i]](1) > c(1) && _vs[ids[i]](2) > c(2))
					lfuVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) >= c(0) && _vs[ids[i]](1) >= c(1) && _vs[ids[i]](2) >= c(2))
					rfuVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) > c(0) && _vs[ids[i]](1) < c(1) && _vs[ids[i]](2) > c(2))
					rbuVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) < c(0) && _vs[ids[i]](1) < c(1) && _vs[ids[i]](2) > c(2))
					lbuVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) < c(0) && _vs[ids[i]](1) > c(1) && _vs[ids[i]](2) < c(2))
					lfdVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) > c(0) && _vs[ids[i]](1) > c(1) && _vs[ids[i]](2) < c(2))
					rfdVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) > c(0) && _vs[ids[i]](1) < c(1) && _vs[ids[i]](2) < c(2))
					rbdVs.push_back(ids[i]);
				else if (_vs[ids[i]](0) <= c(0) && _vs[ids[i]](1) <= c(1) && _vs[ids[i]](2) <= c(2))
					lbdVs.push_back(ids[i]);
			}
			if (!lfuVs.empty())
				lfu = new Node(lfuVs, Vector3(_lbd(0), c(1), c(2)), Vector3(c(0), _rfu(1), _rfu(2)));
			if (!rfuVs.empty())
				rfu = new Node(rfuVs, c, _rfu);
			if (!rbuVs.empty())
				rbu = new Node(rbuVs, Vector3(c(0), _lbd(1), c(2)), Vector3(_rfu(0), c(1), _rfu(2)));
			if (!lbuVs.empty())
				lbu = new Node(lbuVs, Vector3(_lbd(0), _lbd(1), c(2)), Vector3(c(0), c(1), _rfu(2)));
			if (!lfdVs.empty())
				lfd = new Node(lfdVs, Vector3(_lbd(0), c(1), _lbd(2)), Vector3(c(0), _rfu(1), c(2)));
			if (!rfdVs.empty())
				rfd = new Node(rfdVs, Vector3(c(0), c(1), _lbd(2)), Vector3(_rfu(0), _rfu(1), c(2)));
			if (!rbdVs.empty())
				rbd = new Node(rbdVs, Vector3(c(0), _lbd(1), _lbd(2)), Vector3(_rfu(0), c(1), c(2)));
			if (!lbdVs.empty())
				lbd = new Node(lbdVs, _lbd, c);
		}
		bool getSubNodes(std::vector<Node*>& _nodes)
		{
			if (ids.size() <= 1)
				return false;
			if (lfu)
				_nodes.push_back(lfu);
			if (rfu)
				_nodes.push_back(rfu);
			if (rbu)
				_nodes.push_back(rbu);
			if (lbu)
				_nodes.push_back(lbu);
			if (lfd)
				_nodes.push_back(lfd);
			if (rfd)
				_nodes.push_back(rfd);
			if (rbd)
				_nodes.push_back(rbd);
			if (lbd)
				_nodes.push_back(lbd);
			return true;
		}

		void release()
		{
			// vs.swap(std::vector<V*>());
			// if (isLeaf) {
			//	vs.swap(std::vector<V>());
			//	return;
			// }
			if (lfu)
			{
				lfu->release();
				delete lfu;
			}
			if (rfu)
			{
				rfu->release();
				delete rfu;
			}
			if (rbu)
			{
				rbu->release();
				delete rbu;
			}
			if (lbu)
			{
				lbu->release();
				delete lbu;
			}
			if (lfd)
			{
				lfd->release();
				delete lfd;
			}
			if (rfd)
			{
				rfd->release();
				delete rfd;
			}
			if (rbd)
			{
				rbd->release();
				delete rbd;
			}
			if (lbd)
			{
				lbd->release();
				delete lbd;
			}
		}
		void release2()
		{
			if (lfu)
				delete lfu;
			if (rfu)
				delete rfu;
			if (rbu)
				delete rbu;
			if (lbu)
				delete lbu;
			if (lfd)
				delete lfd;
			if (rfd)
				delete rfd;
			if (rbd)
				delete rbd;
			if (lbd)
				delete lbd;
		}

		bool hit(const Vector3 &_v)
		{
			if (isLeaf)
				return true;
			if (_v(0) < c(0) && _v(1) > c(1) && _v(2) > c(2))
			{
				if (lfu)
					return lfu->hit(_v);
				else
					return false;
			}
			else if (_v(0) >= c(0) && _v(1) >= c(1) && _v(2) >= c(2))
			{
				if (rfu)
					return rfu->hit(_v);
				else
					return false;
			}
			else if (_v(0) > c(0) && _v(1) < c(1) && _v(2) > c(2))
			{
				if (rbu)
					return rbu->hit(_v);
				else
					return false;
			}
			else if (_v(0) < c(0) && _v(1) < c(1) && _v(2) > c(2))
			{
				if (lbu)
					return lbu->hit(_v);
				else
					return false;
			}
			else if (_v(0) < c(0) && _v(1) > c(1) && _v(2) < c(2))
			{
				if (lfd)
					return lfd->hit(_v);
				else
					return false;
			}
			else if (_v(0) > c(0) && _v(1) > c(1) && _v(2) < c(2))
			{
				if (rfd)
					return rfd->hit(_v);
				else
					return false;
			}
			else if (_v(0) > c(0) && _v(1) < c(1) && _v(2) < c(2))
			{
				if (rbd)
					return rbd->hit(_v);
				else
					return false;
			}
			else if (_v(0) <= c(0) && _v(1) <= c(1) && _v(2) <= c(2))
			{
				if (lbd)
					return lbd->hit(_v);
				else
					return false;
			}
			return false;
		}
	};

	OcTree() {};
	~OcTree() {};

	void build(const std::vector<Vector3> &_vs)
	{
		if (root)
		{
			root->release();
			delete root;
			root = nullptr;
		}
		if (_vs.empty())
			return;
		vs = _vs;
		Vector3 lbd = _vs[0];
		Vector3 rfu = _vs[0];
		int len = _vs.size();
		std::vector<int> ids(len, 0);
		for (int i = 1; i < len; ++i)
		{
			ids[i] = i;
			if (lbd(0) > _vs[i](0))
				lbd(0) = _vs[i](0);
			else if (rfu(0) < _vs[i](0))
				rfu(0) = _vs[i](0);
			if (lbd(1) > _vs[i](1))
				lbd(1) = _vs[i](1);
			else if (rfu(1) < _vs[i](1))
				rfu(1) = _vs[i](1);
			if (lbd(2) > _vs[i](2))
				lbd(2) = _vs[i](2);
			else if (rfu(2) < _vs[i](2))
				rfu(2) = _vs[i](2);
		}
		root = new Node();
		root->build(_vs, ids, lbd, rfu);
	}
	void build2(const std::vector<Vector3>& _vs)
	{
		if (nodes.size())
		{
			for(int i=0;i<nodes.size();++i)
				nodes[i]->release();
			delete nodes[0];
			nodes.clear();
		}
		if (_vs.empty())
			return;
		vs = _vs;
		Vector3 lbd = _vs[0];
		Vector3 rfu = _vs[0];
		int len = _vs.size();
		std::vector<int> ids(len, 0);
		for (int i = 1; i < len; ++i)
		{
			ids[i] = i;
			if (lbd(0) > _vs[i](0))
				lbd(0) = _vs[i](0);
			else if (rfu(0) < _vs[i](0))
				rfu(0) = _vs[i](0);
			if (lbd(1) > _vs[i](1))
				lbd(1) = _vs[i](1);
			else if (rfu(1) < _vs[i](1))
				rfu(1) = _vs[i](1);
			if (lbd(2) > _vs[i](2))
				lbd(2) = _vs[i](2);
			else if (rfu(2) < _vs[i](2))
				rfu(2) = _vs[i](2);
		}
		Node* nodeRoot = new Node(_vs, ids, lbd, rfu);
		std::vector<Node*> nodesCurrent;
		nodesCurrent.push_back(nodeRoot);
		nodes.push_back(nodeRoot);
		std::vector<Node*> nodesNext;
		//bool needDevide = true;
		while (!nodesCurrent.empty()) {
			nodesNext.clear();
			//needDevide = false;
			for (int i = 0; i < nodesCurrent.size(); ++i) {
				nodesCurrent[i]->getSubNodes(nodesNext);
			}
			nodesCurrent = nodesNext;
			nodes.insert(nodes.end(), nodesNext.begin(), nodesNext.end());
		}
	}

	void release()
	{
		if (root)
		{
			root->release();
			delete root;
		}
	}

	bool hit(const Vector3 &_v)
	{
		if (root)
			return root->hit(_v);
		return false;
	}

	void toBaseTriMesh(std::vector<Eigen::Vector3f> &_ps, std::vector<Eigen::Vector3i> &_faces)
	{
		_ps.clear();
		_faces.clear();
		if (root == nullptr)
			return;
		std::queue<Node *> nodes;
		nodes.push(root);
		Eigen::Vector3f pTmp;
		int s = nodes.size();
		while (!nodes.empty())
		{
			/* code */
			s = nodes.size();
			for (int i = 0; i < s; ++i)
			{
				/* code */
				Node *node = nodes.front();
				nodes.pop();
				pTmp = node->c.cast<float>();
				_ps.push_back(pTmp);
				if (node->isLeaf)
					continue;
				if (node->lfu)
					nodes.push_back(node->lfu);
				if (node->rfu)
					nodes.push_back(node->rfu);
				if (node->rbu)
					nodes.push_back(node->rbu);
				if (node->lbu)
					nodes.push_back(node->lbu);
				if (node->lfd)
					nodes.push_back(node->lfd);
				if (node->rfd)
					nodes.push_back(node->rfd);
				if (node->rbd)
					nodes.push_back(node->rbd);
				if (node->lbd)
					nodes.push_back(node->lbd);
			}
		}
	}

private:
	std::vector<Vector3> vs;
	Node* root = nullptr;
	std::vector<Node*> nodes;
};

using OcTreed = OcTree<double>;
using OcTreef = OcTree<float>;

NSP_SLAM_LYJ_MATH_END

#endif // !SLAM_LYJ_OCTREE_H
