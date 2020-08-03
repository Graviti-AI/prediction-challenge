#include <vector>
using namespace std;

class Node
{
public:
	Node()
	{
		this->s_ = 0;
		this->v_ = 0;
		this->layer_ = 0;
		vector<double > path;
		this->path_ = path;
		this->path_cost_ = 0;
		this->heuristic_cost_ = 0;
		this->total_cost_ = 0;
	}
	Node(const double s, const double v, const int layer, const float path_cost, const float heuristic_cost, const float total_cost) :
		s_(s),
		v_(v),
		layer_(layer),
		path_cost_(path_cost),
		heuristic_cost_(heuristic_cost),
		total_cost_(total_cost)
	{
	}

    double s_;
    double v_;
	int layer_;
	vector<double > path_;
	float path_cost_;
	/// heuristic cost itself -> goal
	float heuristic_cost_;
	/// estimated total cost from start -> goal passing through itself
	float total_cost_;
};

class MinHeap
{
private:
	void BubbleDown(const int index);
	void BubbleUp(const int index);
	void Heapify();

public:
	std::vector<Node> _vector;
	MinHeap(const Node& node);
	MinHeap(){};

	void Insert(const Node& newNode);
	Node GetMin();
	void DeleteMin();
	int GetSize();
};
