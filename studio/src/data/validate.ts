// Field checks for untrusted JSON: the system graph from the bridge
// (system.ts) and the launcher's session manifest (data/manifest.ts). Each
// check raises the bare complaint; `parsing` names the document it was about.
// Caps are the caller's, so a document's bounds stay where they are stated.

export const fail = (what: string): never => {
  throw Error(what);
};

export function parsing<T>(document: string, body: () => T): T {
  try {
    return body();
  } catch (e) {
    throw Error(
        `Invalid ${document}: ${e instanceof Error ? e.message : String(e)}`);
  }
}

export function record(value: unknown, what: string): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    fail(`${what} must be an object`);
  return value as Record<string, unknown>;
}

export function boundedString(source: Record<string, unknown>, key: string,
                              cap: number): string {
  const value = source[key];
  if (typeof value !== 'string' || value.length > cap)
    fail(`${key} must be a string of at most ${cap} characters`);
  return value as string;
}

export function bool(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

// Absent is read as false rather than refused, so a document from a writer
// that predates the field stays readable; a field that is present still has to
// be a boolean ("true" is a mistake, not a value).
export function flag(source: Record<string, unknown>, key: string): boolean {
  const value = source[key];
  if (value === undefined) return false;
  if (typeof value !== 'boolean') fail(`${key} must be a boolean`);
  return value as boolean;
}

export function list(source: Record<string, unknown>, key: string,
                     cap: number): unknown[] {
  const value = source[key];
  if (!Array.isArray(value) || value.length > cap)
    fail(`${key} must be an array of at most ${cap} entries`);
  return value as unknown[];
}
