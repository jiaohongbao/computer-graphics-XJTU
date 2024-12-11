#include "halfedge.h"

#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::optional;
using std::set;
using std::size_t;
using std::string;
using std::unordered_map;
using std::vector;

void traverse_1_ring(Vertex *v, Vertex *Vnew) {
  Halfedge *h = v->halfedge;
  do {

    h->from = Vnew;
    h = h->inv->next;
  } while (h != v->halfedge);
}

HalfedgeMesh::EdgeRecord::EdgeRecord(
    unordered_map<Vertex *, Matrix4f> &vertex_quadrics, Edge *e)
    : edge(e) {
  (void)vertex_quadrics;
  optimal_pos = Vector3f(0.0f, 0.0f, 0.0f);
  cost = 0.0f;
}

bool operator<(const HalfedgeMesh::EdgeRecord &a,
               const HalfedgeMesh::EdgeRecord &b) {
  return a.cost < b.cost;
}

optional<Edge *> HalfedgeMesh::flip_edge(Edge *e) {

  //(void)e;

  if (e->is_boundary()) {
    return nullptr;
  }
  Halfedge *h_inv = e->halfedge->inv;

  Halfedge *h_2_3 = e->halfedge->next;

  Halfedge *h_3_1 = h_2_3->next;

  Halfedge *h_1_4 = h_inv->next;

  Halfedge *h_4_2 = h_1_4->next;

  Vertex *v3 = h_3_1->from;

  Vertex *v4 = h_4_2->from;

  Face *f1 = e->halfedge->face;

  Face *f2 = h_inv->face;

  e->halfedge->set_neighbors(h_3_1, h_1_4, h_inv, v4, e, f1);

  h_inv->set_neighbors(h_4_2, h_2_3, e->halfedge, v3, e, f2);

  h_2_3->prev = h_4_2;
  h_2_3->next = h_inv;

  h_3_1->prev = e->halfedge;
  h_3_1->next = h_1_4;

  h_4_2->prev = h_inv;
  h_4_2->next = h_2_3;

  h_1_4->prev = h_3_1;
  h_1_4->next = e->halfedge;

  e->halfedge->from = v4;
  h_inv->from = v3;
  v3->halfedge = h_3_1;
  v4->halfedge = h_4_2;

  return e;
}

optional<Vertex *> HalfedgeMesh::split_edge(Edge *e) {
  Halfedge *h_2_1 = e->halfedge;

  Halfedge *h_1_2 = h_2_1->inv;

  Halfedge *h_2_3 = h_2_1->next;

  Halfedge *h_3_1 = h_2_3->next;

  Halfedge *h_1_4 = h_1_2->next;

  Halfedge *h_4_2 = h_1_4->next;

  Vertex *v1 = h_2_1->from;

  Vertex *v2 = h_1_2->from;

  Vertex *v3 = h_3_1->from;

  Vertex *v4 = h_4_2->from;

  Face *f1 = h_2_1->face;

  Face *f2 = h_1_2->face;

  Edge *edge23 = new_edge();

  Edge *edge14 = new_edge();

  Edge *edge24 = new_edge();

  Vertex *Vnew = new_vertex();

  Halfedge *new21 = new_halfedge();
  Halfedge *new22 = new_halfedge();
  Halfedge *new23 = new_halfedge();
  Halfedge *new24 = new_halfedge();
  // Halfedge* h_1_n  = new_halfedge();
  // h_1_n            = h_2_1;
  // Halfedge* h_2_n  = new_halfedge();
  // h_2_n            = h_1_2;
  Halfedge *_42new = new_halfedge();
  Halfedge *_32new = new_halfedge();

  Face *face23 = new_face();
  Face *face14 = new_face();
  // Face* face31 = new_face();
  // face31       = f1;
  // Face* face42 = new_face();
  // face42       = f2;

  Vnew->pos = e->center();

  h_2_1->set_neighbors(new23, h_3_1, new21, v1, e, f1);
  h_1_2->set_neighbors(new24, h_4_2, new22, v2, edge24, f2);

  new21->set_neighbors(h_1_4, _42new, h_2_1, Vnew, e, face14);
  new22->set_neighbors(h_2_3, _32new, h_1_2, Vnew, edge24, face23);
  new23->set_neighbors(h_3_1, h_2_1, _32new, Vnew, edge23, f1);
  new24->set_neighbors(h_4_2, h_1_2, _42new, Vnew, edge14, f2);

  _32new->set_neighbors(new22, h_2_3, new23, v3, edge23, face23);
  _42new->set_neighbors(new21, h_1_4, new24, v4, edge14, face14);

  h_3_1->next = h_2_1;
  h_3_1->prev = new23;
  h_3_1->face = f1;

  h_2_3->prev = new22;
  h_2_3->face = face23;
  h_2_3->next = _32new;

  h_1_4->prev = new21;
  h_1_4->next = _42new;
  h_1_4->face = face14;

  h_4_2->next = h_1_2;
  h_4_2->prev = new24;
  h_4_2->face = f2;

  v1->halfedge = h_2_1;
  v2->halfedge = h_1_2;
  v3->halfedge = _32new;
  v4->halfedge = _42new;
  Vnew->halfedge = new21;

  e->halfedge = h_2_1;
  edge24->halfedge = h_1_2;
  edge23->halfedge = _32new;
  edge14->halfedge = _42new;

  face14->halfedge = h_1_4;
  face23->halfedge = h_2_3;
  f1->halfedge = h_3_1;
  f2->halfedge = h_4_2;

  return Vnew;
}

optional<Vertex *> HalfedgeMesh::collapse_edge(Edge *e) {

  Halfedge *h_1_2 = e->halfedge;
  Halfedge *h_2_1 = h_1_2->inv;
  if (h_1_2->is_boundary() || h_2_1->is_boundary()) {
    return nullptr;
  }
  Halfedge *h_2_3 = h_1_2->next;
  Halfedge *h_3_1 = h_2_3->next;
  Halfedge *h_1_4 = h_2_1->next;
  Halfedge *h_4_2 = h_1_4->next;
  Halfedge *h1 = h_3_1->inv;
  Halfedge *hinv1 = h_2_3->inv;
  Halfedge *hinv2 = h_1_4->inv;
  Halfedge *h2 = h_4_2->inv;

  Face *f1 = h_1_2->face;
  Face *f2 = h_2_1->face;

  Vertex *v1 = h_1_2->from;
  Vertex *v2 = h_2_1->from;
  Vertex *v3 = h_3_1->from;
  Vertex *v4 = h_4_2->from;

  if (h1->is_boundary() || h2->is_boundary()) {
    return nullptr;
  }

  Vertex *Vnew = new_vertex();
  Vnew->pos = e->center();

  traverse_1_ring(v1, Vnew);
  traverse_1_ring(v2, Vnew);

  v3->halfedge = hinv1;
  v4->halfedge = hinv2;

  f1->halfedge = h1;
  f2->halfedge = h2;

  h_3_1->edge->halfedge = h1;
  h_4_2->edge->halfedge = h1;
  Vnew->halfedge = h1;

  erase(h_1_4);
  erase(h_4_2);
  erase(h_2_1);
  erase(h_2_3);
  erase(h_3_1);
  erase(h_1_2);

  erase(e);
  erase(h_2_3->edge);
  erase(h_1_4->edge);

  erase(f1);
  erase(f2);
  erase(v1);
  erase(v2);

  return Vnew;
}

void HalfedgeMesh::loop_subdivide() {
  optional<HalfedgeMeshFailure> check_result = validate();
  if (check_result.has_value()) {
    return;
  }
  logger->info("subdivide object {} (ID: {}) with Loop Subdivision strategy",
               object.name, object.id);
  logger->info("original mesh: {} vertices, {} faces in total", vertices.size,
               faces.size);
  // Each vertex and edge of the original mesh can be associated with a vertex
  // in the new (subdivided) mesh.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions using the connectivity of the original
  // (coarse) mesh. Navigating this mesh will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.
  // We will then assign vertex positions in the new mesh based on the values
  // we computed for the original mesh.

  // Compute new positions for all the vertices in the input mesh using
  // the Loop subdivision rule and store them in Vertex::new_pos.
  //    At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh. Use Vertex::is_new for this.

  // Next, compute the subdivided vertex positions associated with edges, and
  // store them in Edge::new_pos.

  // Next, we're going to split every edge in the mesh, in any order.
  // We're also going to distinguish subdivided edges that came from splitting
  // an edge in the original mesh from new edges by setting the boolean
  // Edge::is_new. Note that in this loop, we only want to iterate over edges of
  // the original mesh. Otherwise, we'll end up splitting edges that we just
  // split (and the loop will never end!) I use a vector to store iterators of
  // original because there are three kinds of edges: original edges, edges
  // split from original edges and newly added edges. The newly added edges are
  // marked with Edge::is_new property, so there is not any other property to
  // mark the edges I just split.

  // Now flip any new edge that connects an old and new vertex.

  // Finally, copy new vertex positions into the Vertex::pos.

  // Once we have successfully subdivided the mesh, set global_inconsistent
  // to true to trigger synchronization with GL::Mesh.
  global_inconsistent = true;
  logger->info("subdivided mesh: {} vertices, {} faces in total", vertices.size,
               faces.size);
  logger->info("Loop Subdivision done");
  logger->info("");
  validate();
}

void HalfedgeMesh::simplify() {
  optional<HalfedgeMeshFailure> check_result = validate();
  if (check_result.has_value()) {
    return;
  }
  logger->info("simplify object {} (ID: {})", object.name, object.id);
  logger->info("original mesh: {} vertices, {} faces", vertices.size,
               faces.size);
  unordered_map<Vertex *, Matrix4f> vertex_quadrics;
  unordered_map<Face *, Matrix4f> face_quadrics;
  unordered_map<Edge *, EdgeRecord> edge_records;
  set<EdgeRecord> edge_queue;

  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in face_quadrics

  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in vertex_quadrics

  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an Edge_Record for each edge and sticking it in the
  //    queue. You may want to use the above PQueue<Edge_Record> for this.

  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.

  logger->info("simplified mesh: {} vertices, {} faces", vertices.size,
               faces.size);
  logger->info("simplification done\n");
  global_inconsistent = true;
  validate();
}

void HalfedgeMesh::isotropic_remesh() {
  optional<HalfedgeMeshFailure> check_result = validate();
  if (check_result.has_value()) {
    return;
  }
  logger->info(
      "remesh the object {} (ID: {}) with strategy Isotropic Remeshing",
      object.name, object.id);
  logger->info("original mesh: {} vertices, {} faces", vertices.size,
               faces.size);
  // Compute the mean edge length.

  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  static const size_t iteration_limit = 5;
  set<Edge *> selected_edges;
  for (size_t i = 0; i != iteration_limit; ++i) {
    // Split long edges.

    // Collapse short edges.

    // Flip edges.

    // Vertex averaging.
  }
  logger->info("remeshed mesh: {} vertices, {} faces\n", vertices.size,
               faces.size);
  global_inconsistent = true;
  validate();
}
