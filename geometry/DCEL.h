struct Vertex;
struct HalfEdge;
struct Face;

struct Vertex
{
	int data;
	HalfEdge* edge;		//one edge with this vertex as head
};

struct HalfEdge
{
	Vertex* tail() const { return next->head; }

	int data;
	Vertex* head;
	Face* face;
	HalfEdge* next;
	HalfEdge* adj;
};

struct Face
{
	bool HasEdge(const Edge* e) const {
		HalfEdge* e = edge;
		do
		{
			if(e->adj->face == f)
				return true;
			e = e->next;
		}
		while(e != edge);
		return false;
	}
	HalfEdge AdjEdge(const Face* f) const {
		HalfEdge* e = edge;
		do
		{
			if(e->adj->face == f)
				return e;
			e = e->next;
		}
		while(e != edge);
		return NULL;
	}
	bool IsAdj(const Face* f) const {
		HalfEdge* e = AdjEdge(f);
		return (e != NULL);
	}

	int data;
	HalfEdge* edge;
};