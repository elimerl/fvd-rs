const fs = require("fs");
const packageFileContent = fs.readFileSync("./pkg/package.json", "utf-8");
const packageJSON = JSON.parse(packageFileContent);
packageJSON.type = "module";
packageJSON.main = packageJSON.module;
packageJSON.name = "@elimerl/fvd-rs";
fs.writeFileSync(
    "./pkg/package.json",
    JSON.stringify(packageJSON, null, 4),
    "utf-8"
);

const name = "fvd_rs";

const content = fs.readFileSync(`./pkg/${name}.js`, "utf8");

const patched = content.replace(
    "export default __wbg_init;",
    `export default function init() {
    return __wbg_init("data:application/wasm;base64,${fs.readFileSync(
        `./pkg/${name}_bg.wasm`,
        "base64url"
    )}");
}`
);
fs.writeFileSync(`./pkg/${name}.js`, patched, "utf8");
