/** Minimal XML writer for generating MJCF. */

export type Attributes = Readonly<Record<string, string | number | undefined>>;

/** An XML element with attributes and child elements. */
export interface XmlElement {
  readonly tag: string;
  readonly attributes: Attributes;
  readonly children: readonly XmlElement[];
}

export function el(
  tag: string,
  attributes: Attributes = {},
  children: readonly XmlElement[] = [],
): XmlElement {
  return { tag, attributes, children };
}

/** Formats a number compactly and reproducibly (no exponent noise like 0.30000000000000004). */
export function num(value: number): string {
  return String(Number(value.toPrecision(10)));
}

/** Formats a vector as a space-separated MJCF attribute value. */
export function vec(values: readonly number[]): string {
  return values.map(num).join(" ");
}

function escape(value: string): string {
  return value.replace(/&/g, "&amp;").replace(/"/g, "&quot;").replace(/</g, "&lt;");
}

/** Serialises an element tree with two-space indentation. */
export function serialize(element: XmlElement, depth = 0): string {
  const pad = "  ".repeat(depth);
  const attrs = Object.entries(element.attributes)
    .filter((entry): entry is [string, string | number] => entry[1] !== undefined)
    .map(([key, value]) => ` ${key}="${escape(typeof value === "number" ? num(value) : value)}"`)
    .join("");
  if (element.children.length === 0) return `${pad}<${element.tag}${attrs}/>`;
  const inner = element.children.map((child) => serialize(child, depth + 1)).join("\n");
  return `${pad}<${element.tag}${attrs}>\n${inner}\n${pad}</${element.tag}>`;
}
