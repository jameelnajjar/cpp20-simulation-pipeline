#!/usr/bin/env python3
"""Minimal PDF generator: converts a text/markdown file to a simple PDF.
Uses only Python standard library. Suitable for producing an HLD.pdf for submission.
"""
import sys
import struct
import zlib

PAGE_WIDTH = 595   # A4 points
PAGE_HEIGHT = 842
MARGIN = 50
FONT_SIZE = 10
HEADING_SIZE_1 = 16
HEADING_SIZE_2 = 13
HEADING_SIZE_3 = 11
LINE_HEIGHT = 14
CODE_INDENT = 20
CHARS_PER_LINE = 95  # approximate for 10pt Courier, within margins


def wrap_text(text, width_chars):
    """Wrap text to fit within width_chars characters."""
    words = text.split(' ')
    lines = []
    cur = ''
    for w in words:
        if len(cur) + len(w) + (1 if cur else 0) <= width_chars:
            cur = (cur + ' ' + w) if cur else w
        else:
            if cur:
                lines.append(cur)
            if len(w) > width_chars:
                # force-split very long words/URLs
                while len(w) > width_chars:
                    lines.append(w[:width_chars])
                    w = w[width_chars:]
                cur = w
            else:
                cur = w
    if cur:
        lines.append(cur)
    return lines if lines else ['']


def pdf_string(s):
    """Encode a string for PDF text showing, escaping special chars."""
    return s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')


def parse_markdown_to_lines(md_text):
    """Very simple markdown to styled line list.
    Returns list of (text, style) tuples.
    style: 'h1','h2','h3','body','code','blank','rule'
    """
    lines = []
    in_code = False
    for raw in md_text.splitlines():
        line = raw.rstrip()
        if line.startswith('```'):
            in_code = not in_code
            if in_code:
                lines.append(('', 'blank'))
            else:
                lines.append(('', 'blank'))
            continue
        if in_code:
            lines.append((line, 'code'))
            continue
        if line.startswith('# ') and not line.startswith('## '):
            lines.append((line[2:].strip(), 'h1'))
        elif line.startswith('## ') and not line.startswith('### '):
            lines.append(('', 'blank'))
            lines.append((line[3:].strip(), 'h2'))
        elif line.startswith('### '):
            lines.append((line[4:].strip(), 'h3'))
        elif line.startswith('---'):
            lines.append(('', 'rule'))
        elif line == '':
            lines.append(('', 'blank'))
        else:
            # strip common markdown inline (bold, italic, backticks, links)
            import re
            t = re.sub(r'\*\*(.+?)\*\*', r'\1', line)
            t = re.sub(r'\*(.+?)\*', r'\1', t)
            t = re.sub(r'`(.+?)`', r'\1', t)
            t = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', t)
            lines.append((t, 'body'))
    return lines


def make_pdf(styled_lines, output_path):
    objects = []  # list of (offset, content_bytes)
    obj_offsets = []

    def add_obj(content):
        idx = len(objects) + 1
        objects.append(content)
        return idx

    # Object 1: Catalog (placeholder, fill in after)
    catalog_idx = add_obj(b'')  # will be replaced
    # Object 2: Pages (placeholder)
    pages_idx = add_obj(b'')

    # Fonts
    font_regular_idx = add_obj(
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica'
        b' /Encoding /WinAnsiEncoding >>'
    )
    font_bold_idx = add_obj(
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold'
        b' /Encoding /WinAnsiEncoding >>'
    )
    font_mono_idx = add_obj(
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Courier'
        b' /Encoding /WinAnsiEncoding >>'
    )

    # Build pages
    page_indices = []
    page_content_indices = []

    # Layout: iterate styled_lines, break into pages
    y = PAGE_HEIGHT - MARGIN
    page_cmds = []
    current_page_lines = []

    def flush_page():
        nonlocal page_cmds
        if not current_page_lines:
            return
        stream_parts = []
        for cmd in current_page_lines:
            stream_parts.append(cmd)
        stream = '\n'.join(stream_parts).encode('latin-1', errors='replace')
        content_idx = add_obj(None)  # placeholder
        objects[content_idx - 1] = (
            f'<< /Length {len(stream)} >>\nstream\n'.encode() +
            stream +
            b'\nendstream'
        )
        page_content_indices.append(content_idx)
        page_idx = add_obj(
            f'<< /Type /Page /Parent {pages_idx} 0 R'
            f' /MediaBox [0 0 {PAGE_WIDTH} {PAGE_HEIGHT}]'
            f' /Contents {content_idx} 0 R'
            f' /Resources << /Font << /F1 {font_regular_idx} 0 R'
            f' /F2 {font_bold_idx} 0 R'
            f' /F3 {font_mono_idx} 0 R >> >> >>'.encode()
        )
        page_indices.append(page_idx)
        current_page_lines.clear()

    def new_page():
        nonlocal y
        flush_page()
        y = PAGE_HEIGHT - MARGIN
        current_page_lines.append('BT')

    def ensure_space(needed_lines=1):
        nonlocal y
        needed = needed_lines * LINE_HEIGHT
        if y - needed < MARGIN:
            current_page_lines.append('ET')
            new_page()

    current_page_lines.append('BT')
    y = PAGE_HEIGHT - MARGIN

    for (text, style) in styled_lines:
        if style == 'blank':
            ensure_space(1)
            y -= LINE_HEIGHT / 2
        elif style == 'rule':
            ensure_space(1)
            current_page_lines.append('ET')
            current_page_lines.append(
                f'0.5 w {MARGIN} {y - 2} m {PAGE_WIDTH - MARGIN} {y - 2} l S'
            )
            y -= LINE_HEIGHT
            current_page_lines.append('BT')
        elif style == 'h1':
            ensure_space(2)
            current_page_lines.append(f'1 0 0 1 {MARGIN} {y} Tm')
            current_page_lines.append(f'/F2 {HEADING_SIZE_1} Tf')
            safe = pdf_string(text)
            current_page_lines.append(f'({safe}) Tj')
            y -= HEADING_SIZE_1 + 6
        elif style == 'h2':
            ensure_space(2)
            current_page_lines.append(f'1 0 0 1 {MARGIN} {y} Tm')
            current_page_lines.append(f'/F2 {HEADING_SIZE_2} Tf')
            safe = pdf_string(text)
            current_page_lines.append(f'({safe}) Tj')
            y -= HEADING_SIZE_2 + 5
        elif style == 'h3':
            ensure_space(2)
            current_page_lines.append(f'1 0 0 1 {MARGIN} {y} Tm')
            current_page_lines.append(f'/F2 {HEADING_SIZE_3} Tf')
            safe = pdf_string(text)
            current_page_lines.append(f'({safe}) Tj')
            y -= HEADING_SIZE_3 + 4
        elif style == 'code':
            ensure_space(1)
            current_page_lines.append(f'1 0 0 1 {MARGIN + CODE_INDENT} {y} Tm')
            current_page_lines.append(f'/F3 {FONT_SIZE - 1} Tf')
            display = text[:110]
            safe = pdf_string(display)
            current_page_lines.append(f'({safe}) Tj')
            y -= LINE_HEIGHT
        else:  # body
            wrapped = wrap_text(text, CHARS_PER_LINE)
            ensure_space(len(wrapped))
            current_page_lines.append(f'/F1 {FONT_SIZE} Tf')
            for wline in wrapped:
                current_page_lines.append(f'1 0 0 1 {MARGIN} {y} Tm')
                safe = pdf_string(wline)
                current_page_lines.append(f'({safe}) Tj')
                y -= LINE_HEIGHT

    current_page_lines.append('ET')
    flush_page()

    # Update Pages object
    kids = ' '.join(f'{i} 0 R' for i in page_indices)
    objects[pages_idx - 1] = (
        f'<< /Type /Pages /Kids [{kids}] /Count {len(page_indices)} >>'.encode()
    )

    # Update Catalog
    objects[catalog_idx - 1] = (
        f'<< /Type /Catalog /Pages {pages_idx} 0 R >>'.encode()
    )

    # Write PDF
    with open(output_path, 'wb') as f:
        f.write(b'%PDF-1.4\n')
        offsets = []
        for content in objects:
            offsets.append(f.tell())
            idx = len(offsets)
            f.write(f'{idx} 0 obj\n'.encode())
            f.write(content)
            f.write(b'\nendobj\n')

        xref_pos = f.tell()
        n = len(objects)
        f.write(f'xref\n0 {n + 1}\n'.encode())
        f.write(b'0000000000 65535 f \n')
        for off in offsets:
            f.write(f'{off:010d} 00000 n \n'.encode())

        f.write(
            f'trailer\n<< /Size {n + 1} /Root {catalog_idx} 0 R >>\n'.encode()
        )
        f.write(f'startxref\n{xref_pos}\n%%EOF\n'.encode())

    print(f'Written: {output_path}')


if __name__ == '__main__':
    md_path = sys.argv[1] if len(sys.argv) > 1 else 'HLD.md'
    out_path = sys.argv[2] if len(sys.argv) > 2 else 'HLD.pdf'
    with open(md_path, 'r', encoding='utf-8') as f:
        md = f.read()
    styled = parse_markdown_to_lines(md)
    make_pdf(styled, out_path)
