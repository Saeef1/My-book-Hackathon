// src/services/rag-api.ts

export interface RagQueryRequest {
  query: string;
  provider?: string;
}

export interface RagQueryResponse {
  answer: string;
  sources: string[];
  provider?: string;
}

export interface RagService {
  query(request: RagQueryRequest): Promise<RagQueryResponse>;
  health(): Promise<{ status: string; collection_exists: boolean }>;
}

class RagApiService implements RagService {
  private baseUrl: string;

  constructor(baseUrl: string = process.env.REACT_APP_RAG_API_URL || 'http://localhost:5000') {
    this.baseUrl = baseUrl;
  }

  async query(request: RagQueryRequest): Promise<RagQueryResponse> {
    const response = await fetch(`${this.baseUrl}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: request.query,
        provider: request.provider || 'cohere',
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  }

  async health(): Promise<{ status: string; collection_exists: boolean }> {
    const response = await fetch(`${this.baseUrl}/health`);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  }
}

export default RagApiService;