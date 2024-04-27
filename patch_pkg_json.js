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
