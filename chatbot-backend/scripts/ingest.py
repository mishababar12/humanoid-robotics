import os
from langchain.text_splitter import RecursiveCharacterTextSplitter
from sentence_transformers import SentenceTransformer
# In a real implementation, you would use the services you created
# from app.services.vector_db import get_qdrant_client, create_collection, upsert_vectors
# from app.services.db import get_db_connection

# This is a conceptual example of a data ingestion pipeline.
# To run this, you would need to have the necessary models and services available.

def get_markdown_files(directory):
    """
    Recursively finds all markdown files in a directory.
    """
    markdown_files = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

def chunk_text(text):
    """
    Chunks a text into smaller pieces.
    """
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        length_function=len,
    )
    chunks = text_splitter.split_text(text)
    return chunks

def get_embeddings(chunks):
    """
    Generates embeddings for a list of text chunks.
    """
    # In a real implementation, you would use a sentence transformer model
    # to generate the embeddings. For this example, we will just return dummy vectors.
    # model = SentenceTransformer('all-MiniLM-L6-v2')
    # embeddings = model.encode(chunks)
    # return embeddings
    return [[0.1] * 384 for _ in chunks]

def ingest_data():
    """
    The main data ingestion pipeline.
    """
    # 1. Get the markdown files from the docs directory
    markdown_files = get_markdown_files("../../my-website/docs/")
    print(f"Found {len(markdown_files)} markdown files.")

    # 2. Read the content of the files and chunk them
    all_chunks = []
    all_payloads = []
    for file_path in markdown_files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
        chunks = chunk_text(content)
        all_chunks.extend(chunks)
        for chunk in chunks:
            all_payloads.append({"source": file_path, "text": chunk})
    print(f"Created {len(all_chunks)} chunks.")

    # 3. Generate embeddings for the chunks
    embeddings = get_embeddings(all_chunks)
    print("Generated embeddings.")

    # 4. Upsert the embeddings into the vector database
    # In a real implementation, you would use your vector_db service here
    # client = get_qdrant_client()
    # collection_name = "textbook_content"
    # create_collection(client, collection_name)
    # upsert_vectors(client, collection_name, embeddings, all_payloads)
    print("Data ingestion complete.")

if __name__ == "__main__":
    ingest_data()