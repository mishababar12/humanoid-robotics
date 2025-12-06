import qdrant_client as qc
import qdrant_client.http.models as models
import os

# This is a conceptual example of how to connect to Qdrant Cloud.
# In a real application, you would get the Qdrant credentials from environment variables.

def get_qdrant_client():
    """
    Establishes a connection to the Qdrant Cloud.
    """
    try:
        client = qc.QdrantClient(
            url=os.environ.get("QDRANT_URL"),
            api_key=os.environ.get("QDRANT_API_KEY"),
        )
        return client
    except Exception as e:
        print(f"Error connecting to Qdrant: {e}")
        return None

def create_collection(client: qc.QdrantClient, collection_name: str):
    """
    Creates a new collection in Qdrant if it does not already exist.
    """
    try:
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created successfully.")
    except Exception as e:
        print(f"Error creating collection: {e}")

def upsert_vectors(client: qc.QdrantClient, collection_name: str, vectors, payloads):
    """
    Upserts vectors into a Qdrant collection.
    """
    try:
        client.upsert(
            collection_name=collection_name,
            points=models.Batch(
                ids=[i for i in range(len(vectors))],
                vectors=vectors,
                payloads=payloads,
            ),
            wait=True,
        )
        print("Vectors upserted successfully.")
    except Exception as e:
        print(f"Error upserting vectors: {e}")

def search(client: qc.QdrantClient, collection_name: str, query_vector, limit=5):
    """
    Searches for similar vectors in a Qdrant collection.
    """
    try:
        hits = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit,
        )
        return hits
    except Exception as e:
        print(f"Error searching for vectors: {e}")
        return None
