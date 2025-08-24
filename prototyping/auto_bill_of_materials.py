# Given a set of parts on McMaster (by URL)
# Create a bill of materials (BOM) for these parts
#
# NOTE:
# McMaster-Carr’s site is heavily dynamic and its Terms of Use restrict automated scraping.
# The logic below is a lightweight, best‑effort HTML parser for educational / internal use.
# If the structure changes or content is loaded via JavaScript, consider:
#  - Manually supplying downloaded HTML to `fetch_data(html_override=...)`
#  - Using a headless browser (e.g. Playwright/Selenium) with explicit throttling
# Always respect robots.txt / ToS and add delays for multiple requests.

# Example URLS:
# https://www.mcmaster.com/91273a425/
# https://www.mcmaster.com/4390N166/
# https://www.mcmaster.com/57155K568/
# https://www.mcmaster.com/91292A194/
# https://www.mcmaster.com/90591A260/
# https://www.mcmaster.com/91292A058/
# https://www.mcmaster.com/91292A152/
# https://www.mcmaster.com/91273A425/

import re
import time
from dataclasses import dataclass
from typing import Dict, Optional, Any, List

import requests
from bs4 import BeautifulSoup


USER_AGENT = (
    "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/122.0 Safari/537.36"
)

REQUEST_TIMEOUT = 10
REQUEST_SLEEP_SECONDS = 1.0  # throttle between requests


def _extract_price(text: str) -> Optional[float]:
    m = re.search(r"\$([\d,]+(?:\.\d+)?)", text)
    if not m:
        return None
    try:
        return float(m.group(1).replace(",", ""))
    except ValueError:
        return None


def _clean_ws(s: str) -> str:
    return re.sub(r"\s+", " ", s).strip()


@dataclass
class McMasterPart:
    url: str
    part_number: str = ""
    data: Dict[str, Any] | None = None

    def __post_init__(self):
        self.part_number = self.extract_part_number()
        # Defer fetching until explicitly requested if needed
        self.data = self.fetch_data()

    def extract_part_number(self) -> str:
        # Example URL: https://www.mcmaster.com/91273A425/
        segs = [s for s in self.url.rstrip("/").split("/") if s]
        # Heuristic: last segment that contains a digit / uppercase letter pattern
        return segs[-1] if segs else ""

    def fetch_data(self, html_override: str | None = None) -> Dict[str, Any]:
        """
        Attempt to fetch & parse part metadata.
        Returns a dictionary with best-effort fields:
          - url
          - part_number
          - full_name
          - description
          - unit_price
          - currency
          - min_quantity
          - materials / finish (best effort)
          - spec_table (dict[str,str])
        If network blocked or parsing fails, returns partial info.
        """
        result: Dict[str, Any] = {
            "url": self.url,
            "part_number": self.part_number,
        }

        html = html_override
        if html is None:
            try:
                resp = requests.get(
                    self.url,
                    headers={"User-Agent": USER_AGENT,
                             "Accept-Language": "en-US,en;q=0.9"},
                    timeout=REQUEST_TIMEOUT,
                )
                if resp.status_code != 200:
                    result["error"] = f"HTTP {resp.status_code}"
                    return result
                html = resp.text
                time.sleep(REQUEST_SLEEP_SECONDS)
            except Exception as e:
                result["error"] = f"request_failed: {e}"
                return result

        try:
            soup = BeautifulSoup(html, "html.parser")
        except Exception as e:
            result["error"] = f"parse_failed: {e}"
            return result

        # Title / full name
        title = soup.find("title")
        if title:
            result["full_name"] = _clean_ws(
                title.text.replace(" | McMaster-Carr", ""))
        else:
            # Fallback: h1 / h2
            for tag in ("h1", "h2"):
                h = soup.find(tag)
                if h and h.text.strip():
                    result["full_name"] = _clean_ws(h.text)
                    break

        # Description heuristics (meta description or nearby text)
        meta_desc = soup.find("meta", attrs={"name": "description"})
        if meta_desc and meta_desc.get("content"):
            result["description"] = _clean_ws(meta_desc["content"])

        # Attempt to find price strings
        text_candidates = []
        for el in soup.find_all(text=True):
            t = el.strip()
            if not t:
                continue
            if "$" in t and any(ch.isdigit() for ch in t):
                text_candidates.append(t)

        price = None
        for t in text_candidates:
            p = _extract_price(t)
            if p is not None:
                price = p
                break
        if price is not None:
            result["unit_price"] = price
            result["currency"] = "USD"

        # Attempt to detect minimum quantity (look for 'Pkg.' or 'Per Pack')
        qty = None
        qty_pattern = re.compile(
            r"(Pack of|Pkg\. of|Per Pack)\s+(\d+)", re.IGNORECASE)
        for t in text_candidates:
            m = qty_pattern.search(t)
            if m:
                qty = int(m.group(2))
                break
        if qty:
            result["min_quantity"] = qty

        # Spec table heuristic: look for table rows
        spec_table = {}
        for table in soup.find_all("table"):
            # gather rows with 2 columns
            for tr in table.find_all("tr"):
                cells = tr.find_all(["td", "th"])
                if len(cells) == 2:
                    k = _clean_ws(cells[0].get_text())
                    v = _clean_ws(cells[1].get_text())
                    if k and v and len(k) < 40:
                        spec_table[k] = v
        if spec_table:
            result["spec_table"] = spec_table

        # Materials / Finish heuristics
        material_keys = [k for k in spec_table.keys() if "Material" in k]
        if material_keys:
            result["materials"] = {k: spec_table[k] for k in material_keys}
        finish_keys = [k for k in spec_table.keys() if "Finish" in k]
        if finish_keys:
            result["finish"] = {k: spec_table[k] for k in finish_keys}

        return result


class BillOfMaterials:
    def __init__(self, parts: List[McMasterPart]):
        self.parts = parts

    def total_cost(self) -> Optional[float]:
        total = 0.0
        any_price = False
        for p in self.parts:
            if p.data and "unit_price" in p.data:
                qty = p.data.get("min_quantity", 1)
                total += p.data["unit_price"] * qty
                any_price = True
        return total if any_price else None

    def __repr__(self) -> str:
        # Prepare table data
        headers = ["Part #", "Name", "Qty", "Unit $", "Line Total $"]
        rows = []
        for p in self.parts:
            part_no = p.part_number or "?"
            name = p.data.get("full_name") if p.data else ""
            if name:
                name = name[:80]  # truncate excessively long names
            qty = (p.data or {}).get("min_quantity", 1)
            unit_price = (p.data or {}).get("unit_price")
            unit_price_str = f"{unit_price:.2f}" if unit_price is not None else "-"
            line_total = (unit_price * qty) if unit_price is not None else None
            line_total_str = f"{line_total:.2f}" if line_total is not None else "-"
            rows.append([part_no, name, str(qty),
                        unit_price_str, line_total_str])

        # Compute column widths
        col_widths = [len(h) for h in headers]
        for r in rows:
            for i, cell in enumerate(r):
                col_widths[i] = max(col_widths[i], len(cell))

        def fmt_row(r):
            return " | ".join(cell.ljust(col_widths[i]) for i, cell in enumerate(r))

        sep = "-+-".join("-" * w for w in col_widths)
        lines = ["Bill Of Materials", fmt_row(headers), sep]
        lines += [fmt_row(r) for r in rows]

        grand_total = self.total_cost()
        if grand_total is not None:
            lines.append(sep)
            lines.append(f"Total Cost: ${grand_total:.2f}")
        return "\n".join(lines)


if __name__ == "__main__":
    sample_urls = [
        "https://www.mcmaster.com/91273A425/",
        "https://www.mcmaster.com/91292A194/",
    ]
    parts = [McMasterPart(u) for u in sample_urls]
    bom = BillOfMaterials(parts)
    print(bom)
