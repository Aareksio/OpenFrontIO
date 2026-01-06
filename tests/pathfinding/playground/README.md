# Pathfinding Playground

Interactive web-based visualization tool for exploring and comparing pathfinding algorithms.

## Usage

Start the server from the project root:

```bash
# With path caching (default)
npx tsx tests/pathfinding/playground/server.ts

# Without path caching
npx tsx tests/pathfinding/playground/server.ts --no-cache
```

Then open http://localhost:5555 in your browser.

## Options

- `--no-cache` - Disable path caching in NavigationSatellite to measure uncached performance
