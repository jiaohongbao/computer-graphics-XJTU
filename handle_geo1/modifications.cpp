
void traverse_1_ring(Vertex *v, Vertex *Vnew) {
  Halfedge *h = v->halfedge;
  do {

    h->from = Vnew;
    h = h->inv->next;
  } while (h != v->halfedge);
}

optional<Edge *> HalfedgeMesh::flip_edge(Edge *e) {

  //(void)e;

  if (e->on_bounday()) {
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

  if (h_1_2->is_boundary() || h_2_1->is_boundary()) {
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
