/**
 * CI Service - GitHub Actions workflow status and dispatch
 */

export interface CIStatus {
  id?: number;
  html_url?: string;
  status?: string;
  conclusion?: string;
  created_at?: string;
  updated_at?: string;
  error?: string;
}

export interface CIJob {
  id: number;
  name: string;
  status: string;
  conclusion?: string;
  steps: Array<{
    name: string;
    status: string;
    conclusion?: string;
  }>;
}

export interface CIJobsResponse {
  runId?: number;
  jobs: CIJob[];
  error?: string;
}

export interface CIConfig {
  allowedTargets: string[];
  workflow: string;
  ref: string;
}

class CIService {
  private getRepo(): string | null {
    return import.meta.env.VITE_CI_PUBLISH_REPO || 
           import.meta.env.VITE_PUBLISH_REPO || 
           import.meta.env.VITE_GITHUB_REPOSITORY || 
           null;
  }

  private getToken(): string | null {
    // Check environment variables (Vite uses VITE_ prefix)
    return import.meta.env.VITE_GITHUB_TOKEN || 
           import.meta.env.VITE_GH_TOKEN || 
           // Fallback to localStorage for runtime configuration
           localStorage.getItem('GITHUB_TOKEN') ||
           localStorage.getItem('GH_TOKEN') ||
           null;
  }

  private getWorkflow(): string {
    return import.meta.env.VITE_CI_WORKFLOW || 
           localStorage.getItem('CI_WORKFLOW') ||
           "publish-prod-registry.yml";
  }

  private getRef(): string {
    return import.meta.env.VITE_CI_REF || 
           localStorage.getItem('CI_REF') ||
           "main";
  }

  /**
   * Get CI workflow status
   */
  async getStatus(): Promise<CIStatus> {
    const repo = this.getRepo();
    const token = this.getToken();
    const workflow = this.getWorkflow();
    const ref = this.getRef();

    if (!repo || !token) {
      return { error: "CI not configured" };
    }

    try {
      const response = await fetch(
        `https://api.github.com/repos/${repo}/actions/workflows/${encodeURIComponent(workflow)}/runs?per_page=1&branch=${encodeURIComponent(ref)}`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            "User-Agent": "nava-os-ci",
            Accept: "application/vnd.github+json",
          },
        }
      );

      if (!response.ok) {
        const text = await response.text();
        return { error: `GitHub: ${response.status} ${text}` };
      }

      const data = await response.json();
      const last = data.workflow_runs?.[0];

      if (!last) {
        return { status: "unknown" };
      }

      return {
        id: last.id,
        html_url: last.html_url,
        status: last.status,
        conclusion: last.conclusion,
        created_at: last.created_at,
        updated_at: last.updated_at,
      };
    } catch (error) {
      return { error: error instanceof Error ? error.message : "Unknown error" };
    }
  }

  /**
   * Get CI workflow jobs
   */
  async getJobs(): Promise<CIJobsResponse> {
    const repo = this.getRepo();
    const token = this.getToken();
    const workflow = this.getWorkflow();
    const ref = this.getRef();

    if (!repo || !token) {
      return { jobs: [], error: "CI not configured" };
    }

    try {
      // Get latest run
      const runsResponse = await fetch(
        `https://api.github.com/repos/${repo}/actions/workflows/${encodeURIComponent(workflow)}/runs?per_page=1&branch=${encodeURIComponent(ref)}`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            "User-Agent": "nava-os-ci",
            Accept: "application/vnd.github+json",
          },
        }
      );

      if (!runsResponse.ok) {
        const text = await runsResponse.text();
        return { jobs: [], error: `GitHub: ${runsResponse.status} ${text}` };
      }

      const runs = await runsResponse.json();
      const last = runs.workflow_runs?.[0];

      if (!last) {
        return { jobs: [] };
      }

      // Get jobs for the run
      const jobsResponse = await fetch(
        `https://api.github.com/repos/${repo}/actions/runs/${last.id}/jobs?per_page=100`,
        {
          headers: {
            Authorization: `Bearer ${token}`,
            "User-Agent": "nava-os-ci",
            Accept: "application/vnd.github+json",
          },
        }
      );

      if (!jobsResponse.ok) {
        const text = await jobsResponse.text();
        return { jobs: [], error: `GitHub: ${jobsResponse.status} ${text}` };
      }

      const jobs = await jobsResponse.json();
      const simplified = (jobs.jobs || []).map((j: any) => ({
        id: j.id,
        name: j.name,
        status: j.status,
        conclusion: j.conclusion,
        steps: (j.steps || []).map((s: any) => ({
          name: s.name,
          status: s.status,
          conclusion: s.conclusion,
        })),
      }));

      return { runId: last.id, jobs: simplified };
    } catch (error) {
      return { jobs: [], error: error instanceof Error ? error.message : "Unknown error" };
    }
  }

  /**
   * Dispatch CI workflow
   */
  async dispatch(target: string = "repo"): Promise<{ ok: boolean; error?: string }> {
    const repo = this.getRepo();
    const token = this.getToken();
    const workflow = this.getWorkflow();
    const ref = this.getRef();

    if (!repo || !token) {
      return { ok: false, error: "CI not configured" };
    }

    try {
      const response = await fetch(
        `https://api.github.com/repos/${repo}/actions/workflows/${encodeURIComponent(workflow)}/dispatches`,
        {
          method: "POST",
          headers: {
            Authorization: `Bearer ${token}`,
            "User-Agent": "nava-os-ci",
            Accept: "application/vnd.github+json",
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ ref, inputs: { target } }),
        }
      );

      if (!response.ok) {
        const text = await response.text();
        return { ok: false, error: `GitHub: ${response.status} ${text}` };
      }

      return { ok: true };
    } catch (error) {
      return { ok: false, error: error instanceof Error ? error.message : "Unknown error" };
    }
  }

  /**
   * Get CI configuration
   */
  getConfig(): CIConfig {
    const allowedEnv = import.meta.env.VITE_CI_ALLOWED_TARGETS || 
                       import.meta.env.VITE_REGISTRY_TARGET ||
                       localStorage.getItem('CI_ALLOWED_TARGETS') ||
                       localStorage.getItem('REGISTRY_TARGET') ||
                       "";
    let allowed: string[] = [];
    
    if (allowedEnv.includes(",")) {
      allowed = allowedEnv.split(",").map(s => s.trim()).filter(Boolean);
    } else if (allowedEnv) {
      allowed = [allowedEnv];
    } else {
      allowed = ["repo", "s3"]; // Default: hide R2 unless explicitly allowed
    }

    return {
      allowedTargets: allowed,
      workflow: this.getWorkflow(),
      ref: this.getRef(),
    };
  }
}

export const ciService = new CIService();

