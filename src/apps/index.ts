// Register all apps
import { register } from "../kernel/registry";
import { manifest as editor } from "./editor/manifest";
import { manifest as terminal } from "./terminal/manifest";
import { manifest as simulation } from "./simulation/manifest";
import { manifest as assistant } from "./assistant/manifest";
import { manifest as marketplace } from "./marketplace/manifest";
import { manifest as settings } from "./settings/manifest";
import { manifest as sandboxIframe } from "./sandbox-iframe/manifest";
import { manifest as ciIndicator } from "./ci-indicator/manifest";

// Register apps
register(editor);
register(terminal);
register(simulation);
register(assistant);
register(marketplace);
register(settings);
register(sandboxIframe);
register(ciIndicator);

// Register dock widgets (optional - import to register)
// Uncomment to enable:
// import '../../widgets/perf-monitor';
// import '../../apps/ci-indicator/DockButton'; // CI Indicator as dock widget

// Import command palette (registers widget and commands)
import './command';

export { editor, terminal, simulation, assistant, marketplace, settings, sandboxIframe, ciIndicator };

