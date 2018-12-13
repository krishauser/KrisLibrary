#ifndef HIERARCHICAL_CLUSTERING_H
#define HIERARCHICAL_CLUSTERING_H

#include "statistics.h"

namespace Statistics {

class HierarchicalClustering
{
 public:
	struct distances
	{
		Real dist;
		int index;
	};

	// comparing operator for struct distances
	struct Cmp{
		bool operator()(const distances d1, const distances d2)
			const 
		{
			if(d1.dist == d2.dist)
			{
				return d1.index < d2.index;
			}
			return d1.dist < d2.dist;
		}
	};

	enum Type { SingleLinkage, CompleteLinkage, AverageLinkage };

	///Builds the clustering using standard Euclidean distance
	void Build(const std::vector<Vector>& data,int K,Type type = SingleLinkage);

	///Builds the clustering for a custom distance matrix
	void Build(const Matrix& dist,int K,Type type = SingleLinkage);

	///Returns the indices of the data elements in the i'th cluster
	const std::vector<int>& Cluster(int i) const { return A[i]; }

	// 2d vector for storing lists of items in clusters
	std::vector<std::vector<int> > A;

};



} //namespace Statistics

#endif
