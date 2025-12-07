// Register all apps
import { register } from "../kernel/registry";

// Only import apps that exist
import { manifest as univarmStarter } from "./univarm-starter/manifest";
import { manifest as univarmAdvanced } from "./univarm-advanced/manifest";

// Register apps
register(univarmStarter);
register(univarmAdvanced);

console.log('[Apps] Registered Univarm apps');

export { univarmStarter, univarmAdvanced };

