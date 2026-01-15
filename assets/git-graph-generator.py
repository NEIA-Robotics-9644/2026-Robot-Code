#!/usr/bin/env python3
import os
import re
import subprocess
import sys


def git(repo_root, *args):
    return subprocess.check_output(["git", "-C", repo_root, *args], text=True).strip()


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    exclude = {"refactor-lib", "refactor-robot-vs-lib"}

    try:
        branches = [
            b
            for b in git(
                repo_root,
                "for-each-ref",
                "--format=%(refname:short)",
                "refs/heads",
            ).splitlines()
            if b and b not in exclude
        ]
        current = git(repo_root, "rev-parse", "--abbrev-ref", "HEAD")
    except subprocess.CalledProcessError as exc:
        print(f"Git error: {exc}", file=sys.stderr)
        return 1

    preferred = ["2026-upgrade", "main", "sensor-testing"]
    lane_order = [b for b in preferred if b in branches]
    for b in branches:
        if b not in lane_order:
            lane_order.append(b)

    colors = {
        "2026-upgrade": "#1f77b4",
        "main": "#cb007b",
        "sensor-testing": "#9467bd",
    }
    palette = ["#1f77b4", "#cb007b", "#9467bd", "#8c564b", "#2ca02c", "#ff7f0e"]
    for i, b in enumerate(lane_order):
        if b not in colors:
            colors[b] = palette[i % len(palette)]

    if branches:
        log_lines = git(
            repo_root,
            "log",
            "--topo-order",
            "--max-count=60",
            "--pretty=format:%H %P",
            *branches,
        ).splitlines()
    else:
        log_lines = []

    commits = [line.split()[0] for line in log_lines if line.split()]
    parents_map = {line.split()[0]: line.split()[1:] for line in log_lines if line.split()}

    branch_commits = {b: set(git(repo_root, "rev-list", b).splitlines()) for b in branches}
    main_first_parent = (
        set(git(repo_root, "log", "--first-parent", "--pretty=format:%H", "main").splitlines())
        if "main" in branches
        else set()
    )
    main_commits = branch_commits.get("main", set())

    lane_index = {b: i for i, b in enumerate(lane_order)}
    distance_cache = {}

    def distance_to_tip(branch, sha):
        key = (branch, sha)
        if key not in distance_cache:
            distance_cache[key] = int(git(repo_root, "rev-list", "--count", f"{sha}..{branch}"))
        return distance_cache[key]

    def choose_branch(sha):
        if sha in main_first_parent:
            return "main"
        candidates = [b for b in branches if b != "main" and sha in branch_commits.get(b, set())]
        if candidates:
            candidates.sort(key=lambda b: (distance_to_tip(b, sha), lane_index.get(b, 0), b))
            return candidates[0]
        if sha in main_commits:
            return "main"
        if current in branch_commits and sha in branch_commits[current]:
            return current
        for b in lane_order:
            if sha in branch_commits.get(b, set()):
                return b
        return lane_order[0] if lane_order else "main"

    branch_for = {sha: choose_branch(sha) for sha in commits}

    node_radius = 6.12
    lane_spacing = 40.0
    commit_spacing = 55.0
    margin = 10.0
    corner_radius = 6.0

    positions = {}
    for idx, sha in enumerate(commits):
        lane = lane_index.get(branch_for[sha], 0)
        x = margin + node_radius + lane_spacing * lane
        y = margin + node_radius + commit_spacing * idx
        positions[sha] = (x, y)

    lane_width = margin * 2 + node_radius * 2 + lane_spacing * max(len(lane_order) - 1, 0)
    legend_x = lane_width + 20.0
    legend_step = 18.0
    max_label = max((len(b) for b in lane_order), default=0)
    legend_width = max(100.0, 7.0 * max_label + 30.0)
    width = lane_width + 20.0 + legend_width
    height = margin * 2 + node_radius * 2 + commit_spacing * max(len(commits) - 1, 0)

    def safe_id(text):
        return re.sub(r"[^a-zA-Z0-9_-]", "-", text)

    parts = []
    parts.append("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>")
    parts.append(
        f"<svg width=\"{width:.2f}pt\" height=\"{height:.2f}pt\" viewBox=\"0.00 0.00 {width:.2f} {height:.2f}\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\">"
    )

    parts.append("<defs>")
    marker_ids = {}
    for b in lane_order:
        color = colors[b]
        marker_id = f"arrow-{safe_id(b)}"
        marker_ids[b] = marker_id
        parts.append(
            f"<marker id=\"{marker_id}\" markerWidth=\"6\" markerHeight=\"6\" refX=\"5\" refY=\"3\" orient=\"auto\" markerUnits=\"strokeWidth\">"
        )
        parts.append(f"<path d=\"M0,0 L6,3 L0,6 z\" fill=\"{color}\" />")
        parts.append("</marker>")
    parts.append("</defs>")
    parts.append("")

    parts.append("<g id=\"edges\">")
    for sha in commits:
        x1, y1 = positions[sha]
        color = colors[branch_for[sha]]
        marker = f"url(#{marker_ids[branch_for[sha]]})"
        for parent in parents_map.get(sha, []):
            if parent not in positions:
                continue
            x2, y2 = positions[parent]
            dx = x2 - x1
            dy = y2 - y1
            if abs(dx) < 1e-6:
                sx, sy = x1, y1 + node_radius
                ex, ey = x2, y2 - node_radius
                d = f"M{sx:.2f},{sy:.2f} L{ex:.2f},{ey:.2f}"
            elif abs(dy) < 1e-6:
                sx = x1 + (node_radius if dx > 0 else -node_radius)
                sy = y1
                ex = x2 - (node_radius if dx > 0 else -node_radius)
                ey = y2
                d = f"M{sx:.2f},{sy:.2f} L{ex:.2f},{ey:.2f}"
            else:
                sx = x1 + (node_radius if dx > 0 else -node_radius)
                sy = y1
                ex = x2
                ey = y2 - node_radius
                dx2 = ex - sx
                dy2 = ey - sy
                r = min(corner_radius, abs(dx2), abs(dy2))
                if r <= 1e-6:
                    d = f"M{sx:.2f},{sy:.2f} L{ex:.2f},{ey:.2f}"
                else:
                    pre_x = ex - (r if dx2 > 0 else -r)
                    pre_y = sy
                    post_x = ex
                    post_y = sy + (r if dy2 > 0 else -r)
                    d = (
                        f"M{sx:.2f},{sy:.2f} L{pre_x:.2f},{pre_y:.2f} "
                        f"Q{ex:.2f},{sy:.2f} {post_x:.2f},{post_y:.2f} L{ex:.2f},{ey:.2f}"
                    )
            parts.append(
                f"<path fill=\"none\" stroke=\"{color}\" stroke-width=\"1.5\" stroke-linecap=\"round\" stroke-linejoin=\"round\" d=\"{d}\" marker-start=\"{marker}\" />"
            )
    parts.append("</g>")
    parts.append("")

    parts.append("<g id=\"nodes\">")
    for sha in commits:
        x, y = positions[sha]
        color = colors[branch_for[sha]]
        parts.append("<g class=\"node\">")
        parts.append(f"<title>{sha}</title>")
        parts.append(
            f"<circle cx=\"{x:.2f}\" cy=\"{y:.2f}\" r=\"{node_radius:.2f}\" fill=\"{color}\" stroke=\"{color}\" stroke-width=\"1.5\" />"
        )
        parts.append("</g>")
    parts.append("</g>")
    parts.append("")

    legend_y = margin + 10.0
    parts.append(
        "<g id=\"legend\" font-family=\"Arial, Helvetica, sans-serif\" font-size=\"10\" fill=\"#D0D0D0\">"
    )
    for i, b in enumerate(lane_order):
        cy = legend_y + i * legend_step
        color = colors[b]
        parts.append(
            f"<circle cx=\"{legend_x:.2f}\" cy=\"{cy:.2f}\" r=\"4\" fill=\"{color}\" stroke=\"{color}\" stroke-width=\"1\" />"
        )
        parts.append(f"<text x=\"{legend_x + 10:.2f}\" y=\"{cy + 3:.2f}\">{b}</text>")
    parts.append("</g>")
    parts.append("")

    parts.append("</svg>")

    output_path = os.path.join(repo_root, "assets", "git-graph.svg")
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(parts) + "\n")

    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
