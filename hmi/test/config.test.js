/** Configuration layering and endpoint construction (AGENTS.md §5.7). */

import assert from 'node:assert/strict';
import { describe, it } from 'node:test';

import {
  DEFAULTS,
  encodeTopicParam,
  endpointsFor,
  loadServedConfig,
  overridesFromQuery,
  resolveConfig,
  streamUrl,
} from '../js/config.js';

describe('resolveConfig', () => {
  it('falls back to defaults with no served config and no query', () => {
    assert.deepEqual(resolveConfig(), { ...DEFAULTS });
  });

  it('lets the served config override defaults', () => {
    const config = resolveConfig({ served: { rosbridgePort: 9999 } });
    assert.equal(config.rosbridgePort, 9999);
    assert.equal(config.videoPort, DEFAULTS.videoPort);
  });

  it('lets the query string override the served config', () => {
    const config = resolveConfig({
      served: { host: 'from-nginx', rosbridgePort: 9090 },
      query: { host: 'aquila.local', rosbridgePort: '9091' },
    });
    assert.equal(config.host, 'aquila.local');
    assert.equal(config.rosbridgePort, 9091);
  });

  it('rejects a non-numeric port instead of producing ws://host:NaN', () => {
    const config = resolveConfig({ query: { rosbridgePort: 'nine-oh-nine-oh' } });
    assert.equal(config.rosbridgePort, DEFAULTS.rosbridgePort);
  });

  it('rejects a zero or negative port', () => {
    assert.equal(resolveConfig({ query: { videoPort: '0' } }).videoPort, DEFAULTS.videoPort);
    assert.equal(resolveConfig({ query: { videoPort: '-1' } }).videoPort, DEFAULTS.videoPort);
  });
});

describe('overridesFromQuery', () => {
  it('picks up only the recognised keys', () => {
    const overrides = overridesFromQuery('?host=bench&videoPort=8085&nope=1');
    assert.deepEqual(overrides, { host: 'bench', videoPort: '8085' });
  });

  it('returns nothing for an empty search string', () => {
    assert.deepEqual(overridesFromQuery(''), {});
    assert.deepEqual(overridesFromQuery(undefined), {});
  });
});

describe('endpointsFor', () => {
  const page = { hostname: 'workstation', protocol: 'http:' };

  it('derives the host from the page when config.host is empty', () => {
    const endpoints = endpointsFor(resolveConfig(), page);
    assert.equal(endpoints.rosbridge, 'ws://workstation:9090');
    assert.equal(endpoints.video, 'http://workstation:8080');
  });

  it('honours an explicit host', () => {
    const config = resolveConfig({ served: { host: 'aquila.local' } });
    assert.equal(endpointsFor(config, page).rosbridge, 'ws://aquila.local:9090');
  });

  it('upgrades to wss/https when the page itself is served over TLS', () => {
    const endpoints = endpointsFor(resolveConfig(), {
      hostname: 'aquila.local',
      protocol: 'https:',
    });
    assert.equal(endpoints.rosbridge, 'wss://aquila.local:9090');
    assert.equal(endpoints.video, 'https://aquila.local:8080');
  });

  it('falls back to localhost for a file:// page with no hostname', () => {
    const endpoints = endpointsFor(resolveConfig(), { hostname: '', protocol: 'file:' });
    assert.equal(endpoints.rosbridge, 'ws://localhost:9090');
  });
});

describe('encodeTopicParam', () => {
  it('keeps slashes literal', () => {
    // Measured 24/08/2026 against web-video-server 3.1.0: it does not
    // percent-decode `topic`, and an encoded one answers 200 with 22 bytes and
    // no stream. Both spellings look healthy from the status code alone.
    assert.equal(encodeTopicParam('/demo/camera/image_raw'), '/demo/camera/image_raw');
  });

  it('still escapes characters that would break the query', () => {
    assert.equal(encodeTopicParam('/a b&c=d'), '/a%20b%26c%3Dd');
  });
});

describe('streamUrl', () => {
  it('builds a web_video_server mjpeg URL', () => {
    const url = new URL(streamUrl('http://host:8080', '/demo/camera/image_raw'));
    assert.equal(url.pathname, '/stream');
    assert.equal(url.searchParams.get('topic'), '/demo/camera/image_raw');
    assert.equal(url.searchParams.get('type'), 'mjpeg');
  });

  it('does NOT percent-encode the slashes in the topic', () => {
    const url = streamUrl('http://host:8080', '/demo/camera/image_raw');
    assert.ok(url.includes('topic=/demo/camera/image_raw'), url);
    assert.ok(!url.includes('%2F'), url);
  });

  it('adds a cache-buster only when a nonce is given', () => {
    assert.equal(streamUrl('http://h:8080', '/t').includes('_='), false);
    assert.equal(streamUrl('http://h:8080', '/t', { nonce: 3 }).includes('_=3'), true);
  });
});

describe('loadServedConfig', () => {
  it('returns the parsed body when nginx serves it', async () => {
    const fetchFn = async () => ({ ok: true, json: async () => ({ build: 'abc' }) });
    assert.deepEqual(await loadServedConfig(fetchFn), { build: 'abc' });
  });

  it('returns {} when there is no config.json — the file:// dev path', async () => {
    const fetchFn = async () => {
      throw new Error('failed to fetch');
    };
    assert.deepEqual(await loadServedConfig(fetchFn), {});
  });

  it('returns {} on a non-2xx response rather than throwing into start()', async () => {
    const fetchFn = async () => ({ ok: false, json: async () => ({}) });
    assert.deepEqual(await loadServedConfig(fetchFn), {});
  });
});
