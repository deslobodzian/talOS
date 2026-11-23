// Field checks for untrusted JSON: the system graph from the bridge
// (system.ts), the launcher's session manifest (data/manifest.ts),
// recorded-frame metadata (frames.ts), and the review server's API
// (data/review.ts). Each check raises the bare complaint via `onFail`
// (default: Error); `parsing` names the document it was about. Caps are the
// caller's, so a document's bounds stay where they are stated; data/review.ts
// shares record, bool, and nonNegInt through its ReviewError fail and keeps
// its own uncapped str and array checks with path-qualified messages.

export type Fail = (what: string) => never;

export const fail: Fail = (what: string): never => {
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

export function record(value: unknown, what: string,
                       onFail: Fail = fail): Record<string, unknown> {
  if (!value || typeof value !== 'object' || Array.isArray(value))
    onFail(`${what} must be an object`);
  return value as Record<string, unknown>;
}

export function boundedString(source: Record<string, unknown>, key: string,
                              cap: number, onFail: Fail = fail): string {
  const value = source[key];
  if (typeof value !== 'string' || value.length > cap)
    onFail(`${key} must be a string of at most ${cap} characters`);
  return value as string;
}

export function bool(source: Record<string, unknown>, key: string,
                     onFail: Fail = fail, label = key): boolean {
  const value = source[key];
  if (typeof value !== 'boolean') onFail(`${label} must be a boolean`);
  return value as boolean;
}

// Absent is read as false rather than refused, so a document from a writer
// that predates the field stays readable; a field that is present still has to
// be a boolean ("true" is a mistake, not a value).
export function flag(source: Record<string, unknown>, key: string,
                     onFail: Fail = fail, label = key): boolean {
  const value = source[key];
  if (value === undefined) return false;
  if (typeof value !== 'boolean') onFail(`${label} must be a boolean`);
  return value as boolean;
}

export function list(source: Record<string, unknown>, key: string,
                     cap: number, onFail: Fail = fail): unknown[] {
  const value = source[key];
  if (!Array.isArray(value) || value.length > cap)
    onFail(`${key} must be an array of at most ${cap} entries`);
  return value as unknown[];
}

export function nonNegInt(value: unknown, what: string,
                          onFail: Fail = fail): number {
  if (typeof value !== 'number' || !Number.isSafeInteger(value) || value < 0)
    onFail(`${what} must be a non-negative integer`);
  return value as number;
}
