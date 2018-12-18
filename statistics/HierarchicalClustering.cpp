#include "HierarchicalClustering.h"
#include <set>

using namespace Statistics;
using namespace std;

void HierarchicalClustering::Build(const vector<Vector>& data,int K,Type type)
{
	// fill distances matrix C, then I and P
	int N = (int)data.size();
	int i, j;
	A.clear();
	A.resize(N);
	// multiset for storing sorted distances: insert/delete operation - O(log N)
	std::vector<std::multiset<distances, Cmp> > P(N);
	// 2d vector for storing distances matrix
	std::vector<std::vector<distances> > C(N);
	// vector for storing flags for marking currently active clusters
	std::vector<int> II(N);

	// complexity is: O(N*N*log(N))
	for(i=0; i<N; ++i)
	{
		std::vector<distances> V_temp;
		distances D_temp;
		V_temp.clear();
		std::multiset<distances, Cmp> Q_temp;
		Q_temp.clear();
		V_temp.resize(N);
		for(j=0; j<N; ++j)
		{
		  D_temp.dist = data[i].distance(data[j]);
		  D_temp.index = j;
		  V_temp[j]=D_temp;
		  if(j!=i)  {
		    Q_temp.insert(D_temp);
		  }
		}
		C[i]=V_temp;
		II[i]=1;
		P[i]=Q_temp;

		A[i].resize(1);
		A[i][0] = i;
	}

	// next: build clusters until K clusters left
	// complexity is: O(N*N*log(N))
	for(int n=0; n<N-K; ++n)
	{
		double min_dist = Inf;
		int min_index = 0;
		for(int k=0; k<N-1; ++k)
		{
			if(II[k]==1)
			{
				if(P[k].begin()->dist<min_dist)
				{
					min_dist = P[k].begin()->dist;
					min_index = P[k].begin()->index;
				}
			}
		}
		// we have minimum distance
		// k1, k2 - indexes of most nearest clusters
		int k1 = min_index;
		int k2 = P[k1].begin()->index;
		
		int N_k1 = A[k1].size();
		int N_k2 = A[k2].size();
		
		P[k1].clear();
		// add cluster k2 to A[k1]
		for(size_t l=0;l<A[k2].size();++l)
		{
			A[k1].push_back(A[k2][l]);
		}
		
		// clear the second cluster
		II[k2] = 0;
		// O(N*log(N))
		for(int m=0; m<N; ++m)
		{
			// O(log(N)): insert, erase operations
			if((II[m]!=0)&&(m!=k1))
			{
				P[m].erase(C[m][k1]);
				P[m].erase(C[m][k2]);
				if(type == SingleLinkage)
				{
					C[m][k1].dist = C[m][k1].dist<C[m][k2].dist ? C[m][k1].dist : C[m][k2].dist;
					C[k1][m].dist = C[m][k1].dist;
				}
				else if(type == CompleteLinkage)
				{
					C[m][k1].dist = C[m][k1].dist>C[m][k2].dist ? C[m][k1].dist : C[m][k2].dist;
					C[k1][m].dist = C[m][k1].dist;
				}
				else if(type == AverageLinkage)
				{
					int N_m = A[m].size();
					C[m][k1].dist = (N_k1*C[m][k1].dist + N_k2*C[m][k2].dist)/N_m;
				}
				P[m].insert(C[m][k1]);					
				P[k1].insert(C[k1][m]);
			}
		}
	}
	vector<vector<int> > Atemp;
	Atemp.reserve(K);
	for(i=0;i<N;i++) {
	  if(II[i])
	    Atemp.push_back(A[i]);
	}
	assert((int)Atemp.size()==K);
	swap(A,Atemp);
}
