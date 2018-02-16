#include "Grid.h"
using namespace std;

typedef Grid<float,3> GridT;
int main(int argc, char** argv) {
  GridT::Ind dims(10,10,10);
  GridT::Indf scales(1.0, 1.0, 1.0);

  GridT::Indf ind(1.9, 2.2, 1.1);

  GridT g1(dims, scales, GridT::LINEAR);
  g1.linterpNeighbors(ind);

  cout << ind << endl;
  float twt = 0;
  for (int i = 0; i < g1.NNBRS; ++i) {
    cout << g1.neighbors_[i] << " " << g1.weights_[i] << endl;
    twt += g1.weights_[i];
  }
  cout << twt << endl << endl;

  GridT g(dims, scales, GridT::ADJACENT);
  g.adjacentNeighbors(ind);
  twt = 0;
  for (int i = 0; i < g.NNBRS; ++i) {
    cout << g.neighbors_[i] << " " << g.weights_[i] << endl;
    twt += g.weights_[i];
  }
  cout << twt << endl << endl;

  GridT gnn(dims, scales, GridT::NEAREST);
  gnn.nearestNeighbor(ind);
  cout << gnn.neighbors_[0] << " " << gnn.weights_[0] << endl;
}
